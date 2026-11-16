/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Fibre transport backend over a Zephyr UART (interrupt-driven API),
 * shared with tests/fibre-test (same file, renamed namespace).
 *
 * Bridges the asynchronous stream model of the Fibre stack
 * (AsyncStreamSource / AsyncStreamSink) to the UART ISR:
 *
 *   - The UART ISR only moves bytes between the hardware FIFO and an RX ring
 *     buffer (or the pending TX buffer). It never runs Fibre code.
 *   - Stream completion callbacks are deferred to the Fibre event loop
 *     thread via EventLoop::post(), which is safe from ISR context.
 *
 * The backend serves a single point-to-point channel on the UART selected by
 * the `fibre-uart` devicetree alias. A serial link is always "connected", so
 * the channel is reported as found as soon as a domain starts discovery.
 *
 * Not implemented (not needed by this app, which never tears the transport
 * down): cancel_read() / cancel_write() and repeated
 * start_channel_discovery().
 */

#ifndef CAS_SIM_UART_BACKEND_HPP
#define CAS_SIM_UART_BACKEND_HPP

#include <fibre/bufptr.hpp>
#include <fibre/channel_discoverer.hpp>
#include <fibre/event_loop.hpp>
#include <fibre/logging.hpp>

#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

namespace cas_sim {

class UartBackend final : public fibre::Backend {
public:
    fibre::RichStatus init(fibre::EventLoop* event_loop, fibre::Logger logger) final;
    fibre::RichStatus deinit() final;

    void start_channel_discovery(fibre::Domain* domain, const char* specs,
                                 size_t specs_len,
                                 fibre::ChannelDiscoveryContext** handle) final;
    fibre::RichStatus stop_channel_discovery(fibre::ChannelDiscoveryContext* handle) final;

private:
    /* Ring buffer size (power of two). 512 bytes ≈ 44 ms of buffer at
     * 115200 baud; plenty for the legacy protocol's 128 byte packets. */
    static constexpr size_t kRingSize = 512;
    static constexpr size_t kRingMask = kRingSize - 1;

    /* Persistent functors handed to EventLoop::post(). Callback stores a
     * pointer to the functor, so they must outlive the queued event — hence
     * members, not temporaries. */
    struct RxKick {
        UartBackend* self;
        void operator()() const { self->complete_rx(); }
    };
    struct TxDone {
        UartBackend* self;
        void operator()() const { self->finish_tx(); }
    };

    struct Stream final : fibre::AsyncStreamSource, fibre::AsyncStreamSink {
        UartBackend* parent;

        void start_read(fibre::bufptr_t buffer, fibre::TransferHandle* handle,
                        fibre::Callback<void, fibre::ReadResult> completer) final;
        void cancel_read(fibre::TransferHandle transfer_handle) final;
        void start_write(fibre::cbufptr_t buffer, fibre::TransferHandle* handle,
                         fibre::Callback<void, fibre::WriteResult0> completer) final;
        void cancel_write(fibre::TransferHandle transfer_handle) final;
    };

    /* Event loop thread context */
    void complete_rx();
    void finish_tx();

    friend struct Stream;
    friend struct RxKick;
    friend struct TxDone;

    fibre::EventLoop* event_loop_ = nullptr;
    fibre::Logger logger_ = fibre::Logger::none();
    const struct device* dev_ = nullptr;

    Stream stream_{};
    RxKick rx_kick_{this};
    TxDone tx_done_{this};

    /* RX ring buffer. rx_head_ is advanced by the ISR, rx_tail_ by the event
     * loop thread (under irq_lock()). Both are single-word aligned so access
     * is atomic on ARM. */
    volatile size_t rx_head_ = 0;
    volatile size_t rx_tail_ = 0;
    uint8_t rx_ring_[kRingSize];
    size_t rx_dropped_ = 0;

    /* Pending RX read, armed by the Fibre stack. State changes are guarded
     * with irq_lock() because the ISR also inspects these flags. */
    bool rx_active_ = false;
    bool rx_posted_ = false;
    fibre::bufptr_t rx_buf_{nullptr, nullptr};
    fibre::Callback<void, fibre::ReadResult> rx_completer_;

    /* Pending TX write. One outstanding transfer at a time. */
    bool tx_active_ = false;
    const uint8_t* tx_pos_ = nullptr;
    const uint8_t* tx_end_ = nullptr;
    fibre::Callback<void, fibre::WriteResult0> tx_completer_;

    static void uart_isr_(const struct device* dev, void* user_data);
};

} // namespace cas_sim

#endif // CAS_SIM_UART_BACKEND_HPP
