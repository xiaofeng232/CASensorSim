/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Fibre transport backend over a Zephyr UART, see uart_backend.hpp.
 * Shared with tests/fibre-test (same file, renamed namespace).
 */

#include "uart_backend.hpp"

#include <fibre/fibre.hpp>

#include <zephyr/irq.h>

using namespace fibre; // NOLINT -- the Fibre API is used pervasively below

namespace cas_sim {

RichStatus UartBackend::init(EventLoop* event_loop, Logger logger) {
    F_RET_IF(event_loop_, "already initialized");
    F_RET_IF(!event_loop, "invalid argument");

    dev_ = DEVICE_DT_GET(DT_ALIAS(fibre_uart));
    F_RET_IF(!device_is_ready(dev_), "UART device not ready");

    event_loop_ = event_loop;
    logger_ = logger;

    uart_irq_callback_user_data_set(dev_, uart_isr_, this);
    uart_irq_rx_enable(dev_);

    return RichStatus::success();
}

RichStatus UartBackend::deinit() {
    F_RET_IF(!event_loop_, "not initialized");
    if (dev_) {
        uart_irq_rx_disable(dev_);
        uart_irq_tx_disable(dev_);
        uart_irq_callback_user_data_set(dev_, nullptr, nullptr);
    }
    event_loop_ = nullptr;
    logger_ = Logger::none();
    return RichStatus::success();
}

void UartBackend::start_channel_discovery(Domain* domain, const char* specs,
                                          size_t specs_len,
                                          ChannelDiscoveryContext** handle) {
    (void)specs;
    (void)specs_len;

    F_LOG_D(logger_, "starting UART channel discovery");

    /* The backend serves one fixed UART (fibre-uart alias); no per-discovery
     * state, so the handle can point at the backend itself. */
    if (handle) {
        *handle = reinterpret_cast<ChannelDiscoveryContext*>(this);
    }

    /* A point-to-point serial link is always connected: report the channel
     * immediately. Raw byte stream, so packetized = false and no MTU limit
     * beyond the protocol's internal buffer sizes. */
    domain->add_legacy_channels({kFibreOk, &stream_, &stream_, SIZE_MAX, false},
                                "UART");
}

RichStatus UartBackend::stop_channel_discovery(ChannelDiscoveryContext* handle) {
    (void)handle;
    /* Nothing to stop: the UART keeps running. */
    return RichStatus::success();
}

/* --------------------------------------------------------------------------
 * RX path
 *
 * ISR: UART FIFO -> ring buffer; posts a completion event if a read is
 * armed. Event loop thread: pops from the ring buffer into the caller's
 * buffer and completes the read (partial reads are allowed, >= 1 byte).
 */

void UartBackend::Stream::start_read(bufptr_t buffer, TransferHandle* handle,
                                     Callback<void, ReadResult> completer) {
    UartBackend* self = parent;

    if (handle) {
        /* Cancel is not supported (never invoked by this app); the handle is
         * still filled in for API conformance. */
        *handle = reinterpret_cast<TransferHandle>(self);
    }

    unsigned key = irq_lock();
    self->rx_active_ = true;
    self->rx_posted_ = false;
    self->rx_buf_ = buffer;
    self->rx_completer_ = completer;
    bool have_data = (self->rx_head_ != self->rx_tail_);
    irq_unlock(key);

    if (have_data) {
        /* Data is already buffered: schedule completion without waiting for
         * the next UART interrupt. rx_posted_ makes this idempotent. */
        key = irq_lock();
        bool need = !self->rx_posted_;
        self->rx_posted_ = true;
        irq_unlock(key);
        if (need) {
            (void)self->event_loop_->post(Callback<void>{self->rx_kick_});
        }
    }
}

void UartBackend::complete_rx() {
    unsigned key = irq_lock();
    rx_posted_ = false;

    size_t n = 0;
    if (rx_active_) {
        uint8_t* dst = rx_buf_.begin();
        size_t max = rx_buf_.size();
        while (n < max && rx_tail_ != rx_head_) {
            dst[n++] = rx_ring_[rx_tail_];
            rx_tail_ = (rx_tail_ + 1) & kRingMask;
        }
    }
    irq_unlock(key);

    if (!rx_active_ || n == 0) {
        return; // spurious wakeup or nothing to deliver yet
    }

    rx_active_ = false;
    rx_completer_.invoke_and_clear(ReadResult{kStreamOk, rx_buf_.begin() + n});
}

/* --------------------------------------------------------------------------
 * TX path
 *
 * The Fibre stack hands over a buffer; the ISR feeds it into the UART FIFO
 * in chunks and posts a completion event once everything was accepted by
 * the hardware.
 */

void UartBackend::Stream::start_write(cbufptr_t buffer, TransferHandle* handle,
                                      Callback<void, WriteResult0> completer) {
    UartBackend* self = parent;

    if (handle) {
        *handle = reinterpret_cast<TransferHandle>(self);
    }

    unsigned key = irq_lock();
    F_LOG_IF(self->logger_, self->tx_active_, "concurrent UART write");
    self->tx_active_ = true;
    self->tx_pos_ = buffer.begin();
    self->tx_end_ = buffer.end();
    self->tx_completer_ = completer;
    irq_unlock(key);

    uart_irq_tx_enable(self->dev_);
}

void UartBackend::finish_tx() {
    /* tx_active_ was already cleared by the ISR before posting. */
    tx_completer_.invoke_and_clear(WriteResult0{kStreamOk, tx_end_});
}

void UartBackend::Stream::cancel_read(TransferHandle transfer_handle) {
    (void)transfer_handle;
    /* Not supported; this app never cancels an in-progress transfer. */
}

void UartBackend::Stream::cancel_write(TransferHandle transfer_handle) {
    (void)transfer_handle;
    /* Not supported; this app never cancels an in-progress transfer. */
}

/* --------------------------------------------------------------------------
 * UART ISR
 */

void UartBackend::uart_isr_(const struct device* dev, void* user_data) {
    UartBackend* self = static_cast<UartBackend*>(user_data);

    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        if (uart_irq_rx_ready(dev)) {
            uint8_t tmp[32];
            int n;
            while ((n = uart_fifo_read(dev, tmp, sizeof(tmp))) > 0) {
                for (int i = 0; i < n; i++) {
                    size_t next = (self->rx_head_ + 1) & kRingMask;
                    if (next == self->rx_tail_) {
                        /* Ring full: drop the byte. The protocol layer
                         * detects the damage via the packet CRC. */
                        self->rx_dropped_++;
                        continue;
                    }
                    self->rx_ring_[self->rx_head_] = tmp[i];
                    self->rx_head_ = next;
                }
            }

            unsigned key = irq_lock();
            bool need = self->rx_active_ && !self->rx_posted_;
            if (need) {
                self->rx_posted_ = true;
            }
            irq_unlock(key);
            if (need) {
                (void)self->event_loop_->post(Callback<void>{self->rx_kick_});
            }
        }

        if (uart_irq_tx_ready(dev)) {
            size_t rem = self->tx_end_ - self->tx_pos_;
            if (rem > 0) {
                int n = uart_fifo_fill(dev, self->tx_pos_, rem);
                if (n > 0) {
                    self->tx_pos_ += n;
                    rem = self->tx_end_ - self->tx_pos_;
                }
            }
            if (rem == 0 && self->tx_active_) {
                uart_irq_tx_disable(dev);
                unsigned key = irq_lock();
                self->tx_active_ = false;
                irq_unlock(key);
                (void)self->event_loop_->post(Callback<void>{self->tx_done_});
            }
        }
    }
}

} // namespace cas_sim
