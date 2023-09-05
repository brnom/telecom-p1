#include "uart.hpp"

void UART_RX::put_samples(const unsigned int *buffer, unsigned int n)
{
    int bit_num = 0;
    FILE *dc = fopen("../dc.raw", "wb");

    static enum class State {
        IDLE,
        SEARCHING,
        RECEIVING
    } state = State::IDLE;

    for (int i = 0; i < n; i++)
    {
        int mod = i % (SAMPLES_PER_SYMBOL + 1);
        int sample = buffer[i];
        ring_buffer[mod] = sample;

        switch (state)
        {
        case State::IDLE:
            if (sample == 0)
            {
                state = State::SEARCHING;
                bit_counter = 0;
                bit_counter_noisy_tolerance = 0;
            }
            break;

        case State::SEARCHING:
            bit_counter++;

            if (sample == 0)
            {
                if (bit_counter >= 30)
                {
                    state = State::RECEIVING;
                    received_byte = 0;
                    bit_counter_noisy_tolerance = 0;

                    for (int j = (mod - (bit_counter % SAMPLES_PER_SYMBOL)); j != (mod - 1); j = (j + 1) % SAMPLES_PER_SYMBOL)
                    {
                        bit_num++;
                        if (ring_buffer[j] == 1)
                        {
                            bit_counter_noisy_tolerance++;
                        }
                        else
                        {
                            bit_counter_noisy_tolerance = 0;
                        }

                        if (bit_counter_noisy_tolerance > 10)
                        {
                            i = i - (bit_num - bit_counter_noisy_tolerance);
                            break;
                        }
                    }

                    bit_counter_noisy_tolerance = 0;
                }
            }
            else
            {
                bit_counter_noisy_tolerance++;

                if (bit_counter_noisy_tolerance > 5)
                {
                    state = State::IDLE;
                }
            }
            break;

        case State::RECEIVING:
            bit_counter++;

            fwrite(&bit_counter, 1, sizeof(float), dc);

            if ((bit_counter % SAMPLES_PER_SYMBOL - (SAMPLES_PER_SYMBOL / 2)) == 0)
            {
                received_byte |= (sample << (bit_counter / SAMPLES_PER_SYMBOL - 1));

                if (bit_counter >= (9 * SAMPLES_PER_SYMBOL))
                {
                    state = State::IDLE;
                    get_byte(received_byte);
                }
            }
            break;
        }
    }

    fclose(dc);
}

void UART_TX::put_byte(uint8_t byte)
{
    samples_mutex.lock();
    put_bit(0);
    for (int i = 0; i < 8; i++)
    {
        put_bit(byte & 1);
        byte >>= 1;
    }
    put_bit(1);
    samples_mutex.unlock();
}

void UART_TX::get_samples(unsigned int *buffer, unsigned int n)
{
    samples_mutex.lock();
    std::vector<unsigned int>::size_type i = 0;
    while (!samples.empty() && i < n)
    {
        buffer[i++] = samples.front();
        samples.pop_front();
    }
    samples_mutex.unlock();

    while (i < n)
    {
        buffer[i++] = 1;
    }
}

void UART_TX::put_bit(unsigned int bit)
{
    for (int i = 0; i < SAMPLES_PER_SYMBOL; i++)
    {
        samples.push_back(bit);
    }
}
