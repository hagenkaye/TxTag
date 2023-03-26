;;	.device "ATtiny212" 
	.cseg
	.org	0x00
	
;; registers used by the program	
	.def    r_zero = r0             ; always zero
	.def    r_unprotect = r1        ; set to unprotect signature
    .def    r_byte_counter = r18    ; byte counter for sending bytes
    .def    r_battery = r19         ; battery voltage
    .def    r_index = r20           ; index of byte to send
	.def    r_tmp = r16             ; temporary register
	.def    r_tmp_l = r16
	.def    r_tmp_h = r17
    .def    rx = r26                ; temporary register
	.def    rx_l = r26
	.def    rx_h = r27

;; macros
    .macro  unlock
    out     CPU_CCP,r_unprotect
    .endmacro

; reset and interupt vector table
    rjmp    power_on_reset          ; RESET 
    rjmp    not_implemented         ; CRCSCAN_NMI     
    rjmp    not_implemented         ; BOD_VLM
    rjmp    not_implemented         ; PORTA_PORT
    rjmp    not_implemented         ; PORTB_PORT
    rjmp    not_implemented         ; not used???
    rjmp    not_implemented         ; RTC_CNT
    rjmp    start_tx                ; RTC_PIT
    rjmp    not_implemented         ; TCA0_LUNF / TCA0_OVF
    rjmp    not_implemented         ; TCA_HUNF
    rjmp    not_implemented         ; TCA0_LCMP0 / TCA_CMP0
    rjmp    not_implemented         ; TCA0_LCMP1 / TCA_CMP1
    rjmp    not_implemented         ; TCA0_LCMP2 / TCA_CMP2
    rjmp    not_implemented         ; TCB0_INT
    rjmp    not_implemented         ; TCD0_OVF
    rjmp    not_implemented         ; TCD0_TRIG
    rjmp    not_implemented         ; AC0_AC
    rjmp    adc_result_ready        ; ADC0_RESRDY
    rjmp    not_implemented         ; ADC0_WCOMP
    rjmp    not_implemented         ; TWI0_TWIS
    rjmp    not_implemented         ; TWI0_TWIM
    rjmp    not_implemented         ; SPI0_INT
    rjmp    not_implemented         ; USART0_RXC
    rjmp    not_implemented         ; USART0_DRE
    rjmp    send_byte               ; USART0_TXC
    rjmp    not_implemented         ; NVMCTRL_EE

not_implemented:
power_on_reset:
    out     CPU_SREG,r_zero             ; clear status register
    ldi     r_tmp,0xD8                  ; for ccp unprotect registers
    mov     r_unprotect,r_tmp
    
    ldi     r_tmp_l,LOW(INTERNAL_SRAM_END)  ; set stack pointer to top of memory
    ldi     r_tmp_h,HIGH(INTERNAL_SRAM_END)
    out     CPU_SPL,r_tmp_l
    out     CPU_SPH,r_tmp_h

;; configure clock
;; The clock is set to use the internal 20Mhz clock, with a divide by 64
;; prescaler, so the CPU will be running at 312.5Khz

    unlock
    sts     CLKCTRL_MCLKCTRLA,r_zero
    unlock
    ldi     r_tmp,0b00001011            ; clock prescaler enabled, div by 64
    sts     CLKCTRL_MCLKCTRLB,r_tmp   

;; Port A configuration
;; PA0 - not used (used as UPDI)
;; PA1 - Analog in - Battery Voltage
;; PA2 - Digital Out - Enable RF module (high to enable)
;; PA3 - not used  
;; PA4 - not used (n/a on Attiny212)
;; PA5 - not used (n/a on Attiny212)
;; PA6 - Tx Data out (serial interface)
;; PA7 - Digital Out - low to enable battery voltage read

    ldi     rx_l,LOW(PORTA_PIN0CTRL)
    ldi     rx_h,HIGH(PORTA_PIN0CTRL)
    ldi     r_tmp,0x04                  ; disable input

    st      x+,r_tmp                    ; Pin A0 NO ISR, input disabled
    st      x+,r_tmp                    ; Pin A1 NO ISR, input disabled
    st      x+,r_tmp                    ; Pin A2 NO ISR, input disabled
    st      x+,r_tmp                    ; Pin A3 NO ISR, input disabled
    st      x+,r_tmp                    ; Pin A4 NO ISR, input disabled
    st      x+,r_tmp                    ; Pin A5 NO ISR, input disabled
    st      x+,r_tmp                    ; Pin A6 NO ISR, input disabled
    st      x+,r_tmp                    ; Pin A7 NO ISR, input disabled

    
;; configure the PIT to wake up the TxTag every second
    ldi     r_tmp,0x01                  ; select 1Khz clock
    sts     RTC_CLKSEL,r_tmp
wait_pit_init:
    lds     r_tmp,RTC_PITSTATUS
    and     r_tmp,r_tmp
    brne    wait_pit_init
    ldi     r_tmp,0b01001001            ; set PIT to 1 second
    sts     RTC_PITCTRLA,r_tmp
    ldi     r_tmp,0b00000001            ; enable PIT interrupts
    sts     RTC_PITINTCTRL,r_tmp


;; set initial sleep mode to power down - will wake up on PIT
    ldi     r_tmp,0b00000101            ; sleep mode = power down
    sts     SLPCTRL_CTRLA,r_tmp

    sei

loop:
    sleep                               ; main loop just sits in sleep mode, tx is interrupt driven
    rjmp    loop


;; start tx - interrupt on PIT
;;  change sleep mode to stand by so ADC is active
;;  turn on radio
;;  enable battery level check
;;  start ADC for battery
;;  start RTC - 2 ms delay before sending data
start_tx:
    lds     r_tmp,RTC_PITINTFLAGS       ; reset PIT interrupt flag
    sts     RTC_PITINTFLAGS,r_tmp
    ldi     r_tmp,0b00000001            ; sleep mode = idle
    sts     SLPCTRL_CTRLA,r_tmp

;; configure outputs
    ldi     r_tmp,0b11000100            ; Set output pins
    sts     PORTA_DIR,r_tmp             ; all other outputs (these are open collector outputs)
    ldi     r_tmp,0b01000100            ; radio on, tx set to high, bat voltage to low
    sts     PORTA_OUT,r_tmp             

;; configure VREF
    ldi     r_tmp,0b01000100            ; voltage reference set to 1.5V
    sts     VREF_CTRLA,r_tmp
    ldi     r_tmp,0b00000010            ; turn on for ADC
    sts     VREF_CTRLB,r_tmp

;; configure ADC
    ldi     r_tmp,0b10000101            ; run in standby mode, 8 bit, enable
    sts     ADC0_CTRLA,r_tmp
    sts     ADC0_CTRLB,r_zero           ; no accumulation of samples
    ldi     r_tmp,0b01000001            ; reduced capacitance, 78.125 Khz ADC clock
    sts     ADC0_CTRLC,r_tmp
    ldi     r_tmp,0b01100000            ; delay initial sample by 64 cycles
    sts     ADC0_CTRLD,r_tmp
    sts     ADC0_CTRLE,r_zero           ; no window compare
    sts     ADC0_SAMPCTRL,r_zero        ; no increase in sampling time
    ldi     r_tmp,0x01                  ; muxpos = PA1 or AN1
    sts     ADC0_MUXPOS,r_tmp
    ldi     r_tmp,0x01                  ; enable interrupt on result ready
    sts     ADC0_INTCTRL,r_tmp
    ldi     r_tmp,0x01                  ; start ADC sample
    sts     ADC0_COMMAND,r_tmp
    reti

;; battery voltage level result
adc_result_ready:
    lds     r_tmp,ADC0_INTFLAGS         ; reset ADC interrupt flags
    sts     ADC0_INTFLAGS,r_tmp
    lds     r_battery,ADC0_RESL
    ldi     r_tmp,0b11000100            ; bat voltage to high
    sts     PORTA_OUT,r_tmp             

;; setup UART for 4800 baud
;; 312.5Khz * 4 / 4800 = 260
    ldi     r_tmp,LOW(260)
    ldi     r_tmp_h,HIGH(260)
    sts     USART0_BAUDL,r_tmp
    sts     USART0_BAUDH,r_tmp_h
    ldi     r_tmp,0b00001011            ; Asynchronous, 2 stop bits, 8 data bits, no parity
    sts     USART0_CTRLC,r_tmp
    ldi     r_byte_counter,0            ; reset byte counter to zero
    ldi     r_tmp,0b01000000            ; enable Tx only
    sts     USART0_CTRLB,r_tmp
    ldi     r_tmp,0b01000000            ; transmit complete interrupt
    sts     USART0_CTRLA,r_tmp
    ldi     r_tmp,0xAA                  ; send first byte
    sts     USART0_TXDATAL,r_tmp
    reti

send_byte:
    lds     r_tmp,USART0_STATUS         ; reset UART status
    sts     USART0_STATUS,r_tmp
    inc     r_byte_counter
    cpi     r_byte_counter,4           ; max 4 bytes sent
    breq    stop_tx
    mov     r_index,r_byte_counter
    andi    r_index,0x03
    ldi     r_tmp,0xAA
    cpi     r_index,0
    breq    send_next_byte
    ldi     r_tmp,0x55
    cpi     r_index,1
    breq    send_next_byte
    mov     r_tmp,r_zero
    cpi     r_index,2
    breq    send_next_byte
    mov     r_tmp,r_battery
send_next_byte:
    sts     USART0_TXDATAL,r_tmp
    reti

;; stop sending, power down
;;  set sleep mode to power down
stop_tx:
    ldi     r_tmp,0b00000000            ; disable Tx
    sts     USART0_CTRLB,r_tmp
    ldi     r_tmp,0b00000000            ; disable transmit complete interrupt
    sts     USART0_CTRLA,r_tmp
    ldi     r_tmp,0b00000101            ; sleep mode = power down
    sts     SLPCTRL_CTRLA,r_tmp
    ldi     r_tmp,0b01000100            ; Set output pins
    sts     PORTA_DIR,r_tmp
    ldi     r_tmp,0b01000000            ; radio off, tx set to high, bat voltage to Hi-Z
    sts     PORTA_OUT,r_tmp             

    reti


