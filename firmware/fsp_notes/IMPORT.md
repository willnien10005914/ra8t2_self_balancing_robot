# Import checklist — MCK-RA8T2 + this tree

1. Install FSP ≥ 6.5.0 with e² studio.
2. Open / import Renesas Hall FOC sample for RA8T2 (or dual FOC sample then switch sense to Hall).
3. Add project source folders:
   - `../../firmware/src`
   - include path `../../firmware/include`
4. Implement board hooks (weak symbols):
   - `imu_i2c_write` / `imu_i2c_read` → `R_IIC_Master_*`
   - `console_uart_write` / UART RX → `console_rx_byte`
   - `ai_uart_write` / RX → `ai_link_rx_byte`
   - `time_ms_set_port` → GPT / SysTick ms
5. Replace stubs in `motor_port_fsp.c` with `RM_MOTOR_HALL_*` (L/R).
6. In `hal_entry.c` call `app_main()`.
7. Tune motor params + `BALANCE_LQR_K` before free-standing test.

Official docs:
- https://www.renesas.com/en/design-resources/boards-kits/mck-ra8t2
- https://renesas.github.io/fsp/group___m_o_t_o_r___h_a_l_l.html
- https://github.com/renesas/fsp/releases
