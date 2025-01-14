#ifndef BOARD_H
#define BOARD_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*
 * Setup for STMicroelectronics ORGPAL STM32F767VG board.
 */

/*
 * Board identifier.
 */
#define BOARD_ORGPAL_RADAR_STM32F767_VG
#define BOARD_NAME "OrgPal Radar STM32F767VG"

/*
 * The board has an ULPI USB PHY.
 */
#define BOARD_OTG2_USES_ULPI

/*
 * Board oscillators-related settings.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK 32768U
#endif

#define STM32_LSEDRV (3U << 3U)

#if !defined(STM32_HSECLK)
#define STM32_HSECLK 16000000U
#endif

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD 330U

/*
 * MCU type as defined in the ST header.
 */
#define STM32F767xx
#define STM32F767xx

/*
 * IO pins assignments.
 */
#define GPIOA_UART4_TX  0U
#define GPIOA_UART4_RX  1U
#define GPIOA_PIN2      2U
#define GPIOA_SPI_SEL2  3U
#define GPIOA_DAC1_OUT  4U
#define GPIOA_SPI1_SCK  5U
#define GPIOA_SPI1_MISO 6U
#define GPIOA_SPI1_MOSI 7U
#define GPIOA_PIN8      8U
#define GPIOA_PIN9      9U
#define GPIOA_PIN10     10U
#define GPIOA_OTG_FS_DM 11U
#define GPIOA_OTG_FS_DP 12U
#define GPIOA_SWDIO     13U
#define GPIOA_SWCLK     14U
#define GPIOA_JTDI      15U

#define GPIOB_RELAY_CTRL 0U
#define GPIOB_PIN1       1U
#define GPIOB_SPI3_MOSI  2U
#define GPIOB_SWO        3U
#define GPIOB_PIN4       4U
#define GPIOB_PIN5       5U
#define GPIOB_LED1       6U
#define GPIOB_LED2       7U
#define GPIOB_I2C1_SCL   8U
#define GPIOB_I2C1_SDA   9U
#define GPIOB_PIN10      10U
#define GPIOB_PIN11      11U
#define GPIOB_PIN12      12U
#define GPIOB_PIN13      13U
#define GPIOB_PIN14      14U
#define GPIOB_LED3       15U

#define GPIOC_SPI_SEL1    0U
#define GPIOC_SEN_EN1     1U
#define GPIOC_PIN2        2U
#define GPIOC_SPI_SEL0    3U
#define GPIOC_A121_SPI_SS 4U
#define GPIOC_PIN5        5U
#define GPIOC_PIN6        6U
#define GPIOC_PIN7        7U
#define GPIOC_PIN8        8U
#define GPIOC_PIN9        9U
#define GPIOC_SPI3_SCK    10U
#define GPIOC_SPI3_MISO   11U
#define GPIOC_FLASH_CS    12U
#define GPIOC_PIN13       13U
#define GPIOC_PIN14       14U
#define GPIOC_PIN15       15U

#define GPIOD_FLASH_HOLD    0U
#define GPIOD_FLASH_WP      1U
#define GPIOD_PIN2          2U
#define GPIOD_RS485_TERM_DE 3U
#define GPIOD_USART2_DE     4U
#define GPIOD_USART2_TX     5U
#define GPIOD_USART2_RX     6U
#define GPIOD_PIN7          7U
#define GPIOD_PIN8          8U
#define GPIOD_PIN9          9U
#define GPIOD_PIN10         10U
#define GPIOD_PIN11         11U
#define GPIOD_PIN12         12U
#define GPIOD_PIN13         13U
#define GPIOD_PIN14         14U
#define GPIOD_PIN15         15U

#define GPIOE_PIN0     0U
#define GPIOE_IO       1U
#define GPIOE_PIN2     2U
#define GPIOE_PIN3     3U
#define GPIOE_PIN4     4U
#define GPIOE_PIN5     5U
#define GPIOE_PIN6     6U
#define GPIOE_PIN7     7U
#define GPIOE_PIN8     8U
#define GPIOE_PIN9     9U
#define GPIOE_PIN10    10U
#define GPIOE_PIN11    11U
#define GPIOE_SEN_INT1 12U
#define GPIOE_PIN13    13U
#define GPIOE_PIN14    14U
#define GPIOE_PIN15    15U

#define GPIOF_PIN0  0U
#define GPIOF_PIN1  1U
#define GPIOF_PIN2  2U
#define GPIOF_PIN3  3U
#define GPIOF_PIN4  4U
#define GPIOF_PIN5  5U
#define GPIOF_PIN6  6U
#define GPIOF_PIN7  7U
#define GPIOF_PIN8  8U
#define GPIOF_PIN9  9U
#define GPIOF_PIN10 10U
#define GPIOF_PIN11 11U
#define GPIOF_PIN12 12U
#define GPIOF_PIN13 13U
#define GPIOF_PIN14 14U
#define GPIOF_PIN15 15U

#define GPIOG_PIN0  0U
#define GPIOG_PIN1  1U
#define GPIOG_PIN2  2U
#define GPIOG_PIN3  3U
#define GPIOG_PIN4  4U
#define GPIOG_PIN5  5U
#define GPIOG_PIN6  6U
#define GPIOG_PIN7  7U
#define GPIOG_PIN8  8U
#define GPIOG_PIN9  9U
#define GPIOG_PIN10 10U
#define GPIOG_PIN11 11U
#define GPIOG_PIN12 12U
#define GPIOG_PIN13 13U
#define GPIOG_PIN14 14U
#define GPIOG_PIN15 15U

#define GPIOH_PIN0  0U
#define GPIOH_PIN1  1U
#define GPIOH_PIN2  2U
#define GPIOH_PIN3  3U
#define GPIOH_PIN4  4U
#define GPIOH_PIN5  5U
#define GPIOH_PIN6  6U
#define GPIOH_PIN7  7U
#define GPIOH_PIN8  8U
#define GPIOH_PIN9  9U
#define GPIOH_PIN10 10U
#define GPIOH_PIN11 11U
#define GPIOH_PIN12 12U
#define GPIOH_PIN13 13U
#define GPIOH_PIN14 14U
#define GPIOH_PIN15 15U

#define GPIOI_PIN0  0U
#define GPIOI_PIN1  1U
#define GPIOI_PIN2  2U
#define GPIOI_PIN3  3U
#define GPIOI_PIN4  4U
#define GPIOI_PIN5  5U
#define GPIOI_PIN6  6U
#define GPIOI_PIN7  7U
#define GPIOI_PIN8  8U
#define GPIOI_PIN9  9U
#define GPIOI_PIN10 10U
#define GPIOI_PIN11 11U
#define GPIOI_PIN12 12U
#define GPIOI_PIN13 13U
#define GPIOI_PIN14 14U
#define GPIOI_PIN15 15U

#define GPIOJ_PIN0  0U
#define GPIOJ_PIN1  1U
#define GPIOJ_PIN2  2U
#define GPIOJ_PIN3  3U
#define GPIOJ_PIN4  4U
#define GPIOJ_PIN5  5U
#define GPIOJ_PIN6  6U
#define GPIOJ_PIN7  7U
#define GPIOJ_PIN8  8U
#define GPIOJ_PIN9  9U
#define GPIOJ_PIN10 10U
#define GPIOJ_PIN11 11U
#define GPIOJ_PIN12 12U
#define GPIOJ_PIN13 13U
#define GPIOJ_PIN14 14U
#define GPIOJ_PIN15 15U

#define GPIOK_PIN0  0U
#define GPIOK_PIN1  1U
#define GPIOK_PIN2  2U
#define GPIOK_PIN3  3U
#define GPIOK_PIN4  4U
#define GPIOK_PIN5  5U
#define GPIOK_PIN6  6U
#define GPIOK_PIN7  7U
#define GPIOK_PIN8  8U
#define GPIOK_PIN9  9U
#define GPIOK_PIN10 10U
#define GPIOK_PIN11 11U
#define GPIOK_PIN12 12U
#define GPIOK_PIN13 13U
#define GPIOK_PIN14 14U
#define GPIOK_PIN15 15U

/*
 * IO lines assignments
 */
#define LINE_UART4_TX      PAL_LINE(GPIOA, 0U)
#define LINE_UART4_RX      PAL_LINE(GPIOA, 1U)
#define LINE_SPI_SEL2      PAL_LINE(GPIOA, 3U)
#define LINE_DAC1_OUT      PAL_LINE(GPIOA, 4U)
#define LINE_SPI1_SCK      PAL_LINE(GPIOA, 5U)
#define LINE_SPI1_MISO     PAL_LINE(GPIOA, 6U)
#define LINE_SPI1_MOSI     PAL_LINE(GPIOA, 7U)
#define LINE_OTG_FS_DM     PAL_LINE(GPIOA, 11U)
#define LINE_OTG_FS_DP     PAL_LINE(GPIOA, 12U)
#define LINE_SWDIO         PAL_LINE(GPIOA, 13U)
#define LINE_SWCLK         PAL_LINE(GPIOA, 14U)
#define LINE_JTDI          PAL_LINE(GPIOA, 15U)
#define LINE_RELAY_CTRL    PAL_LINE(GPIOB, 0U)
#define LINE_SPI3_MOSI     PAL_LINE(GPIOB, 2U)
#define LINE_SWO           PAL_LINE(GPIOB, 3U)
#define LINE_LED1          PAL_LINE(GPIOB, 6U)
#define LINE_LED2          PAL_LINE(GPIOB, 7U)
#define LINE_I2C1_SCL      PAL_LINE(GPIOB, 8U)
#define LINE_I2C1_SDA      PAL_LINE(GPIOB, 9U)
#define LINE_LED3          PAL_LINE(GPIOB, 15U)
#define LINE_SPI_SEL1      PAL_LINE(GPIOC, 0U)
#define LINE_SEN_EN1       PAL_LINE(GPIOC, 1U)
#define LINE_SPI_SEL0      PAL_LINE(GPIOC, 3U)
#define LINE_A121_SPI_SS   PAL_LINE(GPIOC, 4U)
#define LINE_SPI3_SCK      PAL_LINE(GPIOC, 10U)
#define LINE_SPI3_MISO     PAL_LINE(GPIOC, 11U)
#define LINE_FLASH_CS      PAL_LINE(GPIOC, 12U)
#define LINE_FLASH_HOLD    PAL_LINE(GPIOD, 0U)
#define LINE_FLASH_WP      PAL_LINE(GPIOD, 1U)
#define LINE_RS485_TERM_DE PAL_LINE(GPIOD, 3U)
#define LINE_USART2_DE     PAL_LINE(GPIOD, 4U)
#define LINE_USART2_TX     PAL_LINE(GPIOD, 5U)
#define LINE_USART2_RX     PAL_LINE(GPIOD, 6U)
#define LINE_IO            PAL_LINE(GPIOE, 1U)
#define LINE_SEN_INT1      PAL_LINE(GPIOE, 12U)

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*
By default, STM32F4 pins are configured as inputs, except some JTAG pins which can impact the power
consumption of the device in different power modes because pins are very sensitive to external noise
in input mode I / O.

To avoid extra I/O current, all pins should be configured as analog input(AIN); in this mode the
Schmitt trigger input is disabled, providing zero consumption for each I / O pin.

We recommend that the I / O speed frequency(driving level) be configured at the lowest possible speed
or as an output push - pull configuration, outputting 0 to the ODR.

The user should also disable the MCO pin of the clock output if not used.
*/

// clang-format off

/*
* I/O ports initial setup, this configuration is established soon after reset
* in the initialization code.
* Please refer to the STM32 Reference Manual for details.
*/
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

/*
* GPIOA setup:
*
* PA0  - UART4_TX                  (alternate 8).
* PA1  - UART4_RX                  (alternate 8).
* PA2  - PIN2                      (input pulldown).
* PA3  - SPI_SLE2                  (output pushpull).
* PA4  - DAC1_OUT                  (analog).
* PA5  - SPI1_SCK                  (alternate 5).
* PA6  - SPI1_MISO                 (alternate 5).
* PA7  - SPI1_MOSI                 (alternate 5).
* PA8  - PIN8                      (input pulldown).
* PA9  - PIN9                      (input pulldown).
* PA10 - PIN10                     (input pulldown).
* PA11 - OTG_FS_DM                 (alternate 10).
* PA12 - OTG_FS_DP                 (alternate 10).
* PA13 - SWDIO                     (alternate 0).
* PA14 - SWCLK                     (alternate 0).
* PA15 - JTDI                      (alternate 0).
*/
#define VAL_GPIOA_MODER            (PIN_MODE_ALTERNATE(GPIOA_UART4_TX) | \
                                     PIN_MODE_ALTERNATE(GPIOA_UART4_RX) | \
                                     PIN_MODE_ANALOG(GPIOA_PIN2) | \
                                     PIN_MODE_OUTPUT(GPIOA_SPI_SEL2) | \
                                     PIN_MODE_ANALOG(GPIOA_DAC1_OUT) | \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_SCK) | \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_MISO) | \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_MOSI) | \
                                     PIN_MODE_ANALOG(GPIOA_PIN8) | \
                                     PIN_MODE_ANALOG(GPIOA_PIN9) | \
                                     PIN_MODE_ANALOG(GPIOA_PIN10) | \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DM) | \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DP) | \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) | \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) | \
                                     PIN_MODE_ALTERNATE(GPIOA_JTDI))

#define VAL_GPIOA_OTYPER           (PIN_OTYPE_PUSHPULL(GPIOA_UART4_TX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART4_RX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN2) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOA_SPI_SEL2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_DAC1_OUT) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_SCK) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_MISO) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_MOSI) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOA_PIN8) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOA_PIN9) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOA_PIN10) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DM) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DP) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOA_SWCLK) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_JTDI))

#define VAL_GPIOA_OSPEEDR          (PIN_OSPEED_HIGH(GPIOA_UART4_TX) | \
                                     PIN_OSPEED_HIGH(GPIOA_UART4_RX) | \
                                     PIN_OSPEED_HIGH(GPIOA_PIN2) | \
                                     PIN_OSPEED_HIGH(GPIOA_SPI_SEL2) | \
                                     PIN_OSPEED_VERYLOW(GPIOA_DAC1_OUT) | \
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_SCK) | \
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_MISO) | \
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_MOSI) | \
                                     PIN_OSPEED_HIGH(GPIOA_PIN8) | \
                                     PIN_OSPEED_HIGH(GPIOA_PIN9) | \
                                     PIN_OSPEED_HIGH(GPIOA_PIN10) | \
                                     PIN_OSPEED_HIGH(GPIOA_OTG_FS_DM) | \
                                     PIN_OSPEED_HIGH(GPIOA_OTG_FS_DP) | \
                                     PIN_OSPEED_HIGH(GPIOA_SWDIO) | \
                                     PIN_OSPEED_HIGH(GPIOA_SWCLK) | \
                                     PIN_OSPEED_HIGH(GPIOA_JTDI))

#define VAL_GPIOA_PUPDR            (PIN_PUPDR_FLOATING(GPIOA_UART4_TX) | \
                                     PIN_PUPDR_FLOATING(GPIOA_UART4_RX) | \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN2) | \
                                     PIN_PUPDR_PULLDOWN(GPIOA_SPI_SEL2) | \
                                     PIN_PUPDR_FLOATING(GPIOA_DAC1_OUT) | \
                                     PIN_PUPDR_FLOATING(GPIOA_SPI1_SCK) | \
                                     PIN_PUPDR_FLOATING(GPIOA_SPI1_MISO) | \
                                     PIN_PUPDR_FLOATING(GPIOA_SPI1_MOSI) | \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN8) | \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN9) | \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN10) | \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DM) | \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DP) | \
                                     PIN_PUPDR_PULLUP(GPIOA_SWDIO) | \
                                     PIN_PUPDR_PULLDOWN(GPIOA_SWCLK) | \
                                     PIN_PUPDR_FLOATING(GPIOA_JTDI))

#define VAL_GPIOA_ODR              (PIN_ODR_HIGH(GPIOA_UART4_TX) | \
                                     PIN_ODR_HIGH(GPIOA_UART4_RX) | \
                                     PIN_ODR_HIGH(GPIOA_PIN2) | \
                                     PIN_ODR_HIGH(GPIOA_SPI_SEL2) | \
                                     PIN_ODR_HIGH(GPIOA_DAC1_OUT) | \
                                     PIN_ODR_HIGH(GPIOA_SPI1_SCK) | \
                                     PIN_ODR_HIGH(GPIOA_SPI1_MISO) | \
                                     PIN_ODR_HIGH(GPIOA_SPI1_MOSI) | \
                                     PIN_ODR_HIGH(GPIOA_PIN8) | \
                                     PIN_ODR_HIGH(GPIOA_PIN9) | \
                                     PIN_ODR_HIGH(GPIOA_PIN10) | \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DM) | \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DP) | \
                                     PIN_ODR_HIGH(GPIOA_SWDIO) | \
                                     PIN_ODR_HIGH(GPIOA_SWCLK) | \
                                     PIN_ODR_HIGH(GPIOA_JTDI))

#define VAL_GPIOA_AFRL             (PIN_AFIO_AF(GPIOA_UART4_TX, 8U) | \
                                     PIN_AFIO_AF(GPIOA_UART4_RX, 8U) | \
                                     PIN_AFIO_AF(GPIOA_PIN2, 0U) | \
                                     PIN_AFIO_AF(GPIOA_SPI_SEL2, 0U) | \
                                     PIN_AFIO_AF(GPIOA_DAC1_OUT, 0U) | \
                                     PIN_AFIO_AF(GPIOA_SPI1_SCK, 5U) | \
                                     PIN_AFIO_AF(GPIOA_SPI1_MISO, 5U) | \
                                     PIN_AFIO_AF(GPIOA_SPI1_MOSI, 5U))

#define VAL_GPIOA_AFRH             (PIN_AFIO_AF(GPIOA_PIN8, 0U) | \
                                     PIN_AFIO_AF(GPIOA_PIN9, 0U) | \
                                     PIN_AFIO_AF(GPIOA_PIN10, 0U) | \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DM, 10U) | \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DP, 10U) | \
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0U) | \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0U) | \
                                     PIN_AFIO_AF(GPIOA_JTDI, 0U))

/*
* GPIOB setup:
*
* PB0  - RELAY_CTRL                (output pushpull).
* PB1  - PIN1                      (input pulldown).
* PB2  - SPI3_MOSI                 (alternate 7).
* PB3  - SWO                       (alternate 0).
* PB4  - PIN4                      (input pulldown).
* PB5  - PIN5                      (input pulldown).
* PB6  - LED1                      (output pushpull).
* PB7  - LED2                      (output pushpull).
* PB8  - I2C1_SCL                  (input floating).
* PB9  - I2C1_SDA                  (input floating).
* PB10 - PIN10                     (input pulldown).
* PB11 - PIN11                     (input pulldown).
* PB12 - PIN12                     (input pulldown).
* PB13 - PIN13                     (input pulldown).
* PB14 - PIN14                     (input pulldown).
* PB15 - LED3                      (output pushpull).
*/

#define VAL_GPIOB_MODER             (PIN_MODE_OUTPUT(GPIOB_RELAY_CTRL) | \
                                     PIN_MODE_ANALOG(GPIOB_PIN1) |       \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI3_MOSI) | \
                                     PIN_MODE_ALTERNATE(GPIOB_SWO) |     \
                                     PIN_MODE_ANALOG(GPIOB_PIN4) |        \
                                     PIN_MODE_ANALOG(GPIOB_PIN5) |        \
                                     PIN_MODE_OUTPUT(GPIOB_LED1) |       \
                                     PIN_MODE_OUTPUT(GPIOB_LED2) |       \
                                     PIN_MODE_INPUT(GPIOB_I2C1_SCL) |    \
                                     PIN_MODE_INPUT(GPIOB_I2C1_SDA) |    \
                                     PIN_MODE_ANALOG(GPIOB_PIN10) |       \
                                     PIN_MODE_ANALOG(GPIOB_PIN11) |       \
                                     PIN_MODE_ANALOG(GPIOB_PIN12) |       \
                                     PIN_MODE_ANALOG(GPIOB_PIN13) |       \
                                     PIN_MODE_ANALOG(GPIOB_PIN14) |       \
                                     PIN_MODE_OUTPUT(GPIOB_LED3))

#define VAL_GPIOB_OTYPER            (PIN_OTYPE_OPENDRAIN(GPIOB_RELAY_CTRL) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_PIN1) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI3_MOSI) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SWO) |         \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_PIN4) |        \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_PIN5) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LED1) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LED2) |        \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SCL) |   \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SDA) |   \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_PIN10) |       \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_PIN11) |       \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_PIN12) |       \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_PIN13) |       \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_PIN14) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LED3))

#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_HIGH(GPIOB_RELAY_CTRL) | \
                                     PIN_OSPEED_HIGH(GPIOB_PIN1) |        \
                                     PIN_OSPEED_HIGH(GPIOB_SPI3_MOSI) |   \
                                     PIN_OSPEED_HIGH(GPIOB_SWO) |         \
                                     PIN_OSPEED_HIGH(GPIOB_PIN4) |        \
                                     PIN_OSPEED_HIGH(GPIOB_PIN5) |        \
                                     PIN_OSPEED_HIGH(GPIOB_LED1) |        \
                                     PIN_OSPEED_HIGH(GPIOB_LED2) |        \
                                     PIN_OSPEED_HIGH(GPIOB_I2C1_SCL) |    \
                                     PIN_OSPEED_HIGH(GPIOB_I2C1_SDA) |    \
                                     PIN_OSPEED_HIGH(GPIOB_PIN10) |       \
                                     PIN_OSPEED_HIGH(GPIOB_PIN11) |       \
                                     PIN_OSPEED_HIGH(GPIOB_PIN12) |       \
                                     PIN_OSPEED_HIGH(GPIOB_PIN13) |       \
                                     PIN_OSPEED_HIGH(GPIOB_PIN14) |       \
                                     PIN_OSPEED_HIGH(GPIOB_LED3))

#define VAL_GPIOB_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOB_RELAY_CTRL) | \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN1) |        \
                                     PIN_PUPDR_FLOATING(GPIOB_SPI3_MOSI) |   \
                                     PIN_PUPDR_FLOATING(GPIOB_SWO) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN4) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN5) |        \
                                     PIN_PUPDR_FLOATING(GPIOB_LED1) |        \
                                     PIN_PUPDR_FLOATING(GPIOB_LED2) |        \
                                     PIN_PUPDR_FLOATING(GPIOB_I2C1_SCL) |    \
                                     PIN_PUPDR_FLOATING(GPIOB_I2C1_SDA) |    \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN10) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN11) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN12) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN13) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN14) |       \
                                     PIN_PUPDR_FLOATING(GPIOB_LED3))

#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_RELAY_CTRL) | \
                                     PIN_ODR_HIGH(GPIOB_PIN1) |        \
                                     PIN_ODR_HIGH(GPIOB_SPI3_MOSI) |   \
                                     PIN_ODR_HIGH(GPIOB_SWO) |         \
                                     PIN_ODR_HIGH(GPIOB_PIN4) |        \
                                     PIN_ODR_HIGH(GPIOB_PIN5) |        \
                                     PIN_ODR_HIGH(GPIOB_LED1) |        \
                                     PIN_ODR_HIGH(GPIOB_LED2) |        \
                                     PIN_ODR_HIGH(GPIOB_I2C1_SCL) |    \
                                     PIN_ODR_HIGH(GPIOB_I2C1_SDA) |    \
                                     PIN_ODR_HIGH(GPIOB_PIN10) |       \
                                     PIN_ODR_HIGH(GPIOB_PIN11) |       \
                                     PIN_ODR_HIGH(GPIOB_PIN12) |       \
                                     PIN_ODR_HIGH(GPIOB_PIN13) |       \
                                     PIN_ODR_HIGH(GPIOB_PIN14) |       \
                                     PIN_ODR_HIGH(GPIOB_LED3))

#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_RELAY_CTRL, 0U) | \
                                     PIN_AFIO_AF(GPIOB_PIN1, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_SPI3_MOSI, 7U) |  \
                                     PIN_AFIO_AF(GPIOB_SWO, 0U) |        \
                                     PIN_AFIO_AF(GPIOB_PIN4, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_PIN5, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_LED1, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_LED2, 0U))

#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_I2C1_SCL, 4U) |       \
                                     PIN_AFIO_AF(GPIOB_I2C1_SDA, 4U) |       \
                                     PIN_AFIO_AF(GPIOB_PIN10, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_PIN11, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_PIN12, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_PIN13, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_PIN14, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_LED3, 0U))

/*
* GPIOC setup:
*
* PC0  - SPI_SEL1                  (output pushpull).
* PC1  - SEN_EN1                   (output pushpull).
* PC2  - PIN2                      (input pulldown).
* PC3  - SPI_SEL0                  (output pushpull).
* PC4  - A121_SPI_SS               (output pushpull).
* PC5  - PIN5                      (input pulldown).
* PC6  - PIN6                      (input pulldown).
* PC7  - PIN7                      (input pulldown).
* PC8  - PIN8                      (input pulldown).
* PC9  - PIN9                      (input pulldown).
* PC10 - SPI3_SCK                  (alternate 5).
* PC11 - SPI3_MISO                 (alternate 5).
* PC12 - FLASH_CS                  (output pushpull).
* PC13 - PIN13                     (input pulldown).
* PC14 - PIN14                     (input pulldown).
* PC15 - PIN15                     (input pulldown).
*/

#define VAL_GPIOC_MODER             (PIN_MODE_OUTPUT(GPIOC_SPI_SEL1) | \
                                     PIN_MODE_OUTPUT(GPIOC_SEN_EN1) | \
                                     PIN_MODE_ANALOG(GPIOC_PIN2) |      \
                                     PIN_MODE_OUTPUT(GPIOC_SPI_SEL0) | \
                                     PIN_MODE_OUTPUT(GPIOC_A121_SPI_SS) | \
                                     PIN_MODE_ANALOG(GPIOC_PIN5) |      \
                                     PIN_MODE_ANALOG(GPIOC_PIN6) |      \
                                     PIN_MODE_ANALOG(GPIOC_PIN7) |      \
                                     PIN_MODE_ANALOG(GPIOC_PIN8) |      \
                                     PIN_MODE_ANALOG(GPIOC_PIN9) |      \
                                     PIN_MODE_ALTERNATE(GPIOC_SPI3_SCK) | \
                                     PIN_MODE_ALTERNATE(GPIOC_SPI3_MISO) | \
                                     PIN_MODE_OUTPUT(GPIOC_FLASH_CS) |  \
                                     PIN_MODE_ANALOG(GPIOC_PIN13) |     \
                                     PIN_MODE_ANALOG(GPIOC_PIN14) |     \
                                     PIN_MODE_ANALOG(GPIOC_PIN15))

#define VAL_GPIOC_OTYPER            (PIN_OTYPE_OPENDRAIN(GPIOC_SPI_SEL1) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_SEN_EN1) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_PIN2) |      \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_SPI_SEL0) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_A121_SPI_SS) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_PIN5) |      \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_PIN6) |      \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_PIN7) |      \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_PIN8) |      \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_PIN9) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SPI3_SCK) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SPI3_MISO) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_FLASH_CS) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_PIN13) |     \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_PIN14) |     \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_PIN15))

#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_HIGH(GPIOC_SPI_SEL1) | \
                                     PIN_OSPEED_HIGH(GPIOC_SEN_EN1) | \
                                     PIN_OSPEED_HIGH(GPIOC_PIN2) |      \
                                     PIN_OSPEED_HIGH(GPIOC_SPI_SEL0) | \
                                     PIN_OSPEED_HIGH(GPIOC_A121_SPI_SS) | \
                                     PIN_OSPEED_HIGH(GPIOC_PIN5) |      \
                                     PIN_OSPEED_HIGH(GPIOC_PIN6) |      \
                                     PIN_OSPEED_HIGH(GPIOC_PIN7) |      \
                                     PIN_OSPEED_HIGH(GPIOC_PIN8) |      \
                                     PIN_OSPEED_HIGH(GPIOC_PIN9) |      \
                                     PIN_OSPEED_HIGH(GPIOC_SPI3_SCK) |  \
                                     PIN_OSPEED_HIGH(GPIOC_SPI3_MISO) | \
                                     PIN_OSPEED_HIGH(GPIOC_FLASH_CS) |  \
                                     PIN_OSPEED_HIGH(GPIOC_PIN13) |     \
                                     PIN_OSPEED_HIGH(GPIOC_PIN14) |     \
                                     PIN_OSPEED_HIGH(GPIOC_PIN15))

#define VAL_GPIOC_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOC_SPI_SEL1) | \
                                     PIN_PUPDR_PULLDOWN(GPIOC_SEN_EN1) | \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN2) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOC_SPI_SEL0) | \
                                     PIN_PUPDR_PULLUP(GPIOC_A121_SPI_SS) | \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN5) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN6) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN7) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN8) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN9) |      \
                                     PIN_PUPDR_FLOATING(GPIOC_SPI3_SCK) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_SPI3_MISO) | \
                                     PIN_PUPDR_FLOATING(GPIOC_FLASH_CS) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN13) |     \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN14) |     \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN15))

#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_SPI_SEL1) | \
                                     PIN_ODR_HIGH(GPIOC_SEN_EN1) | \
                                     PIN_ODR_HIGH(GPIOC_PIN2) |      \
                                     PIN_ODR_HIGH(GPIOC_SPI_SEL0) | \
                                     PIN_ODR_HIGH(GPIOC_A121_SPI_SS) | \
                                     PIN_ODR_HIGH(GPIOC_PIN5) |      \
                                     PIN_ODR_HIGH(GPIOC_PIN6) |      \
                                     PIN_ODR_HIGH(GPIOC_PIN7) |      \
                                     PIN_ODR_HIGH(GPIOC_PIN8) |      \
                                     PIN_ODR_HIGH(GPIOC_PIN9) |      \
                                     PIN_ODR_HIGH(GPIOC_SPI3_SCK) |  \
                                     PIN_ODR_HIGH(GPIOC_SPI3_MISO) | \
                                     PIN_ODR_HIGH(GPIOC_FLASH_CS) |  \
                                     PIN_ODR_HIGH(GPIOC_PIN13) |     \
                                     PIN_ODR_HIGH(GPIOC_PIN14) |     \
                                     PIN_ODR_HIGH(GPIOC_PIN15))

#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_SPI_SEL1, 0U) |    \
                                     PIN_AFIO_AF(GPIOC_SEN_EN1, 0U) |    \
                                     PIN_AFIO_AF(GPIOC_PIN2, 0U) |      \
                                     PIN_AFIO_AF(GPIOC_SPI_SEL0, 0U) | \
                                     PIN_AFIO_AF(GPIOC_A121_SPI_SS, 0U) | \
                                     PIN_AFIO_AF(GPIOC_PIN5, 0U) |      \
                                     PIN_AFIO_AF(GPIOC_PIN6, 0U) |      \
                                     PIN_AFIO_AF(GPIOC_PIN7, 0U))

#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_PIN8, 0U) |      \
                                     PIN_AFIO_AF(GPIOC_PIN9, 0U) |      \
                                     PIN_AFIO_AF(GPIOC_SPI3_SCK, 6U) |  \
                                     PIN_AFIO_AF(GPIOC_SPI3_MISO, 6U) | \
                                     PIN_AFIO_AF(GPIOC_FLASH_CS, 0U) |  \
                                     PIN_AFIO_AF(GPIOC_PIN13, 0U) |     \
                                     PIN_AFIO_AF(GPIOC_PIN14, 0U) |     \
                                     PIN_AFIO_AF(GPIOC_PIN15, 0U))

/*
* GPIOD setup:
*
* PD0  - FLASH_HOLD                (output pushpull).
* PD1  - FLASH_WP                  (output pushpull).
* PD2  - PIN2                      (input pullup).
* PD3  - RS485_TERM_DE             (output pushpull).
* PD4  - USART2_DE                 (alternate 7).
* PD5  - USART2_TX                 (alternate 7).
* PD6  - USART2_RX                 (alternate 7).
* PD7  - PIN7                      (input pullup).
* PD8  - PIN8                      (input pullup).
* PD9  - PIN9                      (input pullup).
* PD10 - PIN10                     (input pullup).
* PD11 - PIN11                     (input pullup).
* PD12 - PIN12                     (input pullup).
* PD13 - PIN13                     (input pullup).
* PD14 - PIN14                     (input pullup).
* PD15 - PIN15                     (input pullup).
*/
#define VAL_GPIOD_MODER             (PIN_MODE_OUTPUT(GPIOD_FLASH_HOLD) |    \
                                     PIN_MODE_OUTPUT(GPIOD_FLASH_WP) |      \
                                     PIN_MODE_ANALOG(GPIOD_PIN2) |          \
                                     PIN_MODE_OUTPUT(GPIOD_RS485_TERM_DE) | \
                                     PIN_MODE_ALTERNATE(GPIOD_USART2_DE) |  \
                                     PIN_MODE_ALTERNATE(GPIOD_USART2_TX) |  \
                                     PIN_MODE_ALTERNATE(GPIOD_USART2_RX) |  \
                                     PIN_MODE_ANALOG(GPIOD_PIN7) |          \
                                     PIN_MODE_ANALOG(GPIOD_PIN8) |          \
                                     PIN_MODE_ANALOG(GPIOD_PIN9) |          \
                                     PIN_MODE_ANALOG(GPIOD_PIN10) |         \
                                     PIN_MODE_ANALOG(GPIOD_PIN11) |         \
                                     PIN_MODE_ANALOG(GPIOD_PIN12) |         \
                                     PIN_MODE_ANALOG(GPIOD_PIN13) |         \
                                     PIN_MODE_ANALOG(GPIOD_PIN14) |         \
                                     PIN_MODE_ANALOG(GPIOD_PIN15))

#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_FLASH_HOLD) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_FLASH_WP) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOD_PIN2) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_RS485_TERM_DE) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_USART2_DE) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_USART2_TX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_USART2_RX) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOD_PIN7) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOD_PIN8) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOD_PIN9) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOD_PIN10) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOD_PIN11) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOD_PIN12) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOD_PIN13) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOD_PIN14) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOD_PIN15))

#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_HIGH(GPIOD_FLASH_HOLD) | \
                                     PIN_OSPEED_HIGH(GPIOD_FLASH_WP) |   \
                                     PIN_OSPEED_HIGH(GPIOD_PIN2) |     \
                                     PIN_OSPEED_HIGH(GPIOD_RS485_TERM_DE) | \
                                     PIN_OSPEED_HIGH(GPIOD_USART2_DE) | \
                                     PIN_OSPEED_HIGH(GPIOD_USART2_TX) | \
                                     PIN_OSPEED_HIGH(GPIOD_USART2_RX) | \
                                     PIN_OSPEED_HIGH(GPIOD_PIN7) |     \
                                     PIN_OSPEED_HIGH(GPIOD_PIN8) |     \
                                     PIN_OSPEED_HIGH(GPIOD_PIN9) |     \
                                     PIN_OSPEED_HIGH(GPIOD_PIN10) |    \
                                     PIN_OSPEED_HIGH(GPIOD_PIN11) |    \
                                     PIN_OSPEED_HIGH(GPIOD_PIN12) |    \
                                     PIN_OSPEED_HIGH(GPIOD_PIN13) |    \
                                     PIN_OSPEED_HIGH(GPIOD_PIN14) |    \
                                     PIN_OSPEED_HIGH(GPIOD_PIN15))

#define VAL_GPIOD_PUPDR             (PIN_PUPDR_PULLUP(GPIOD_FLASH_HOLD) |       \
                                     PIN_PUPDR_PULLUP(GPIOD_FLASH_WP) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN2) |             \
                                     PIN_PUPDR_FLOATING(GPIOD_RS485_TERM_DE) |  \
                                     PIN_PUPDR_FLOATING(GPIOD_USART2_DE) |      \
                                     PIN_PUPDR_FLOATING(GPIOD_USART2_TX) |      \
                                     PIN_PUPDR_FLOATING(GPIOD_USART2_RX) |      \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN7) |             \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN8) |             \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN9) |             \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN10) |            \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN11) |            \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN12) |            \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN13) |            \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN14) |            \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN15))

#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_FLASH_HOLD) |       \
                                     PIN_ODR_HIGH(GPIOD_FLASH_WP) |         \
                                     PIN_ODR_HIGH(GPIOD_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOD_RS485_TERM_DE) |    \
                                     PIN_ODR_HIGH(GPIOD_USART2_DE) |        \
                                     PIN_ODR_HIGH(GPIOD_USART2_TX) |        \
                                     PIN_ODR_HIGH(GPIOD_USART2_RX) |        \
                                     PIN_ODR_HIGH(GPIOD_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN15))

#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_FLASH_HOLD, 0U) |    \
                                     PIN_AFIO_AF(GPIOD_FLASH_WP, 0U) |      \
                                     PIN_AFIO_AF(GPIOD_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_RS485_TERM_DE, 0U) | \
                                     PIN_AFIO_AF(GPIOD_USART2_DE, 7U) |     \
                                     PIN_AFIO_AF(GPIOD_USART2_TX, 7U) |     \
                                     PIN_AFIO_AF(GPIOD_USART2_RX, 7U) |     \
                                     PIN_AFIO_AF(GPIOD_PIN7, 0U))

#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_PIN8, 0U)  |         \
                                     PIN_AFIO_AF(GPIOD_PIN9, 0U)  |         \
                                     PIN_AFIO_AF(GPIOD_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_PIN15, 0U))

/*
* GPIOE setup:
*
* PE0  - PIN0                      (input pullup).
* PE1  - IO1                       (input pulldow).
* PE2  - PIN2                      (input pullup).
* PE3  - PIN3                      (input pullup).
* PE4  - PIN4                      (input pullup).
* PE5  - PIN5                      (input pullup).
* PE6  - PIN6                      (input pullup).
* PE7  - PIN7                      (input pullup).
* PE8  - PIN8                      (input pullup).
* PE9  - PIN9                      (input pullup).
* PE10 - PIN10                     (input pullup).
* PE11 - PIN11                     (input pullup).
* PE12 - INT1                      (input pulldown).
* PE13 - PIN13                     (input pullup).
* PE14 - PIN14                     (input pullup).
* PE15 - PIN15                     (input pullup).
*/

#define VAL_GPIOE_MODER             (PIN_MODE_ANALOG(GPIOE_PIN0) |      \
                                     PIN_MODE_INPUT(GPIOE_IO) |        \
                                     PIN_MODE_ANALOG(GPIOE_PIN2) |      \
                                     PIN_MODE_ANALOG(GPIOE_PIN3) |      \
                                     PIN_MODE_ANALOG(GPIOE_PIN4) |      \
                                     PIN_MODE_ANALOG(GPIOE_PIN5) |      \
                                     PIN_MODE_ANALOG(GPIOE_PIN6) |      \
                                     PIN_MODE_ANALOG(GPIOE_PIN7) |      \
                                     PIN_MODE_ANALOG(GPIOE_PIN8) |      \
                                     PIN_MODE_ANALOG(GPIOE_PIN9) |      \
                                     PIN_MODE_ANALOG(GPIOE_PIN10) |     \
                                     PIN_MODE_ANALOG(GPIOE_PIN11) |     \
                                     PIN_MODE_INPUT(GPIOE_SEN_INT1) |  \
                                     PIN_MODE_ANALOG(GPIOE_PIN13) |     \
                                     PIN_MODE_ANALOG(GPIOE_PIN14) |     \
                                     PIN_MODE_ANALOG(GPIOE_PIN15))

#define VAL_GPIOE_OTYPER            (PIN_OTYPE_OPENDRAIN(GPIOE_PIN0) |       \
                                     PIN_OTYPE_OPENDRAIN(GPIOE_IO) |         \
                                     PIN_OTYPE_OPENDRAIN(GPIOE_PIN2) |       \
                                     PIN_OTYPE_OPENDRAIN(GPIOE_PIN3) |       \
                                     PIN_OTYPE_OPENDRAIN(GPIOE_PIN4) |       \
                                     PIN_OTYPE_OPENDRAIN(GPIOE_PIN5) |       \
                                     PIN_OTYPE_OPENDRAIN(GPIOE_PIN6) |       \
                                     PIN_OTYPE_OPENDRAIN(GPIOE_PIN7) |       \
                                     PIN_OTYPE_OPENDRAIN(GPIOE_PIN8) |       \
                                     PIN_OTYPE_OPENDRAIN(GPIOE_PIN9) |       \
                                     PIN_OTYPE_OPENDRAIN(GPIOE_PIN10) |      \
                                     PIN_OTYPE_OPENDRAIN(GPIOE_PIN11) |      \
                                     PIN_OTYPE_OPENDRAIN(GPIOE_SEN_INT1) |   \
                                     PIN_OTYPE_OPENDRAIN(GPIOE_PIN13) |      \
                                     PIN_OTYPE_OPENDRAIN(GPIOE_PIN14) |      \
                                     PIN_OTYPE_OPENDRAIN(GPIOE_PIN15))

#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_HIGH(GPIOE_PIN0) |     \
                                     PIN_OSPEED_HIGH(GPIOE_IO) |       \
                                     PIN_OSPEED_HIGH(GPIOE_PIN2) |     \
                                     PIN_OSPEED_HIGH(GPIOE_PIN3) |     \
                                     PIN_OSPEED_HIGH(GPIOE_PIN4) |     \
                                     PIN_OSPEED_HIGH(GPIOE_PIN5) |     \
                                     PIN_OSPEED_HIGH(GPIOE_PIN6) |     \
                                     PIN_OSPEED_HIGH(GPIOE_PIN7) |     \
                                     PIN_OSPEED_HIGH(GPIOE_PIN8) |     \
                                     PIN_OSPEED_HIGH(GPIOE_PIN9) |     \
                                     PIN_OSPEED_HIGH(GPIOE_PIN10) |    \
                                     PIN_OSPEED_HIGH(GPIOE_PIN11) |    \
                                     PIN_OSPEED_HIGH(GPIOE_SEN_INT1) | \
                                     PIN_OSPEED_HIGH(GPIOE_PIN13) |    \
                                     PIN_OSPEED_HIGH(GPIOE_PIN14) |    \
                                     PIN_OSPEED_HIGH(GPIOE_PIN15))

#define VAL_GPIOE_PUPDR             (PIN_PUPDR_PULLUP(GPIOE_PIN0) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOE_IO) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN11) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOE_SEN_INT1) |   \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN15))

#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(GPIOE_PIN0) |        \
                                     PIN_ODR_HIGH(GPIOE_IO) |          \
                                     PIN_ODR_HIGH(GPIOE_PIN2) |        \
                                     PIN_ODR_HIGH(GPIOE_PIN3) |        \
                                     PIN_ODR_HIGH(GPIOE_PIN4) |        \
                                     PIN_ODR_HIGH(GPIOE_PIN5) |        \
                                     PIN_ODR_HIGH(GPIOE_PIN6) |        \
                                     PIN_ODR_HIGH(GPIOE_PIN7) |        \
                                     PIN_ODR_HIGH(GPIOE_PIN8) |        \
                                     PIN_ODR_HIGH(GPIOE_PIN9) |        \
                                     PIN_ODR_HIGH(GPIOE_PIN10) |       \
                                     PIN_ODR_HIGH(GPIOE_PIN11) |       \
                                     PIN_ODR_HIGH(GPIOE_SEN_INT1) |    \
                                     PIN_ODR_HIGH(GPIOE_PIN13) |       \
                                     PIN_ODR_HIGH(GPIOE_PIN14) |       \
                                     PIN_ODR_HIGH(GPIOE_PIN15))

#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_IO, 0U) |        \
                                     PIN_AFIO_AF(GPIOE_PIN2, 0U) |      \
                                     PIN_AFIO_AF(GPIOE_PIN3, 0U) |      \
                                     PIN_AFIO_AF(GPIOE_PIN4, 0U) |      \
                                     PIN_AFIO_AF(GPIOE_PIN5, 0U) |      \
                                     PIN_AFIO_AF(GPIOE_PIN6, 0U) |      \
                                     PIN_AFIO_AF(GPIOE_PIN7, 0U) |      \
                                     PIN_AFIO_AF(GPIOE_PIN8, 0U))

#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_PIN9, 0U) |      \
                                     PIN_AFIO_AF(GPIOE_PIN10, 0U) |     \
                                     PIN_AFIO_AF(GPIOE_PIN11, 0U) |     \
                                     PIN_AFIO_AF(GPIOE_SEN_INT1, 0U) |  \
                                     PIN_AFIO_AF(GPIOE_PIN13, 0U) |     \
                                     PIN_AFIO_AF(GPIOE_PIN14, 0U) |     \
                                     PIN_AFIO_AF(GPIOE_PIN15, 0U))

/*
* GPIOF setup:
*   
* PF0  - PIN0                      (input pullup).
* PF1  - PIN1                      (input pullup).
* PF2  - PIN2                      (input pullup).
* PF3  - PIN3                      (input pullup).
* PF4  - PIN4                      (input pullup).
* PF5  - PIN5                      (input pullup).
* PF6  - PIN6                      (input pullup).
* PF7  - PIN7                      (input pullup).
* PF8  - PIN8                      (input pullup).
* PF9  - PIN9                      (input pullup).
* PF10 - PIN10                     (input pullup).
* PF11 - PIN11                     (input pullup).
* PF12 - PIN12                     (input pullup).
* PF13 - PIN13                     (input pullup).
* PF14 - PIN14                     (input pullup).
* PF15 - PIN15                     (input pullup).
*/

#define VAL_GPIOF_MODER             (PIN_MODE_ANALOG(GPIOF_PIN0) |      \
                                     PIN_MODE_ANALOG(GPIOF_PIN1) |      \
                                     PIN_MODE_ANALOG(GPIOF_PIN2) |      \
                                     PIN_MODE_ANALOG(GPIOF_PIN3) |      \
                                     PIN_MODE_ANALOG(GPIOF_PIN4) |      \
                                     PIN_MODE_ANALOG(GPIOF_PIN5) |      \
                                     PIN_MODE_ANALOG(GPIOF_PIN6) |      \
                                     PIN_MODE_ANALOG(GPIOF_PIN7) |      \
                                     PIN_MODE_ANALOG(GPIOF_PIN8) |      \
                                     PIN_MODE_ANALOG(GPIOF_PIN9) |      \
                                     PIN_MODE_ANALOG(GPIOF_PIN10) |     \
                                     PIN_MODE_ANALOG(GPIOF_PIN11) |     \
                                     PIN_MODE_ANALOG(GPIOF_PIN12) |     \
                                     PIN_MODE_ANALOG(GPIOF_PIN13) |     \
                                     PIN_MODE_ANALOG(GPIOF_PIN14) |     \
                                     PIN_MODE_ANALOG(GPIOF_PIN15))

#define VAL_GPIOF_OTYPER            (PIN_OTYPE_OPENDRAIN(GPIOF_PIN0) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOF_PIN1) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOF_PIN2) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOF_PIN3) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOF_PIN4) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOF_PIN5) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOF_PIN6) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOF_PIN7) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOF_PIN8) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOF_PIN9) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOF_PIN10) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOF_PIN11) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOF_PIN12) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOF_PIN13) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOF_PIN14) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOF_PIN15))

#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_HIGH(GPIOF_PIN0) |     \
                                     PIN_OSPEED_HIGH(GPIOF_PIN1) |     \
                                     PIN_OSPEED_HIGH(GPIOF_PIN2) |     \
                                     PIN_OSPEED_HIGH(GPIOF_PIN3) |     \
                                     PIN_OSPEED_HIGH(GPIOF_PIN4) |     \
                                     PIN_OSPEED_HIGH(GPIOF_PIN5) |     \
                                     PIN_OSPEED_HIGH(GPIOF_PIN6) |     \
                                     PIN_OSPEED_HIGH(GPIOF_PIN7) |     \
                                     PIN_OSPEED_HIGH(GPIOF_PIN8) |     \
                                     PIN_OSPEED_HIGH(GPIOF_PIN9) |     \
                                     PIN_OSPEED_HIGH(GPIOF_PIN10) |    \
                                     PIN_OSPEED_HIGH(GPIOF_PIN11) |    \
                                     PIN_OSPEED_HIGH(GPIOF_PIN12) |    \
                                     PIN_OSPEED_HIGH(GPIOF_PIN13) |    \
                                     PIN_OSPEED_HIGH(GPIOF_PIN14) |    \
                                     PIN_OSPEED_HIGH(GPIOF_PIN15))

#define VAL_GPIOF_PUPDR             (PIN_PUPDR_PULLUP(GPIOF_PIN0) |    \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN1) |    \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN2) |    \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN3) |    \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN4) |    \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN5) |    \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN6) |    \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN7) |    \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN8) |    \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN9) |    \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN10) |   \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN11) |   \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN12) |   \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN13) |   \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN14) |   \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN15))

#define VAL_GPIOF_ODR               (PIN_ODR_HIGH(GPIOF_PIN0) |        \
                                     PIN_ODR_HIGH(GPIOF_PIN1) |        \
                                     PIN_ODR_HIGH(GPIOF_PIN2) |        \
                                     PIN_ODR_HIGH(GPIOF_PIN3) |        \
                                     PIN_ODR_HIGH(GPIOF_PIN4) |        \
                                     PIN_ODR_HIGH(GPIOF_PIN5) |        \
                                     PIN_ODR_HIGH(GPIOF_PIN6) |        \
                                     PIN_ODR_HIGH(GPIOF_PIN7) |        \
                                     PIN_ODR_HIGH(GPIOF_PIN8) |        \
                                     PIN_ODR_HIGH(GPIOF_PIN9) |        \
                                     PIN_ODR_HIGH(GPIOF_PIN10) |       \
                                     PIN_ODR_HIGH(GPIOF_PIN11) |       \
                                     PIN_ODR_HIGH(GPIOF_PIN12) |       \
                                     PIN_ODR_HIGH(GPIOF_PIN13) |       \
                                     PIN_ODR_HIGH(GPIOF_PIN14) |       \
                                     PIN_ODR_HIGH(GPIOF_PIN15))

#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_PIN0, 0U) |     \
                                     PIN_AFIO_AF(GPIOF_PIN1, 0U) |     \
                                     PIN_AFIO_AF(GPIOF_PIN2, 0U) |     \
                                     PIN_AFIO_AF(GPIOF_PIN3, 0U) |     \
                                     PIN_AFIO_AF(GPIOF_PIN4, 0U) |     \
                                     PIN_AFIO_AF(GPIOF_PIN5, 0U) |     \
                                     PIN_AFIO_AF(GPIOF_PIN6, 0U) |     \
                                     PIN_AFIO_AF(GPIOF_PIN7, 0U))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_PIN8, 0U) |     \
                                     PIN_AFIO_AF(GPIOF_PIN9, 0U) |     \
                                     PIN_AFIO_AF(GPIOF_PIN10, 0U) |    \
                                     PIN_AFIO_AF(GPIOF_PIN11, 0U) |    \
                                     PIN_AFIO_AF(GPIOF_PIN12, 0U) |    \
                                     PIN_AFIO_AF(GPIOF_PIN13, 0U) |    \
                                     PIN_AFIO_AF(GPIOF_PIN14, 0U) |    \
                                     PIN_AFIO_AF(GPIOF_PIN15, 0U))

/*
* GPIOG setup:
*   
* PG0  - PIN0                      (input pullup).
* PG1  - PIN1                      (input pullup).
* PG2  - PIN2                      (input pullup).
* PG3  - PIN3                      (input pullup).
* PG4  - PIN4                      (input pullup).
* PG5  - PIN5                      (input pullup).
* PG6  - PIN6                      (input pullup).
* PG7  - PIN7                      (input pullup).
* PG8  - PIN8                      (input pullup).
* PG9  - PIN9                      (input pullup).
* PG10 - PIN10                     (input pullup).
* PG11 - PIN11                     (input pullup).
* PG12 - PIN12                     (input pullup).
* PG13 - PIN13                     (input pullup).
* PG14 - PIN14                     (input pullup).
* PG15 - PIN15                     (input pullup).
*/

#define VAL_GPIOG_MODER             (PIN_MODE_ANALOG(GPIOG_PIN0) |      \
                                     PIN_MODE_ANALOG(GPIOG_PIN1) |      \
                                     PIN_MODE_ANALOG(GPIOG_PIN2) |      \
                                     PIN_MODE_ANALOG(GPIOG_PIN3) |      \
                                     PIN_MODE_ANALOG(GPIOG_PIN4) |      \
                                     PIN_MODE_ANALOG(GPIOG_PIN5) |      \
                                     PIN_MODE_ANALOG(GPIOG_PIN6) |      \
                                     PIN_MODE_ANALOG(GPIOG_PIN7) |      \
                                     PIN_MODE_ANALOG(GPIOG_PIN8) |      \
                                     PIN_MODE_ANALOG(GPIOG_PIN9) |      \
                                     PIN_MODE_ANALOG(GPIOG_PIN10) |     \
                                     PIN_MODE_ANALOG(GPIOG_PIN11) |     \
                                     PIN_MODE_ANALOG(GPIOG_PIN12) |     \
                                     PIN_MODE_ANALOG(GPIOG_PIN13) |     \
                                     PIN_MODE_ANALOG(GPIOG_PIN14) |     \
                                     PIN_MODE_ANALOG(GPIOG_PIN15))

#define VAL_GPIOG_OTYPER            (PIN_OTYPE_OPENDRAIN(GPIOG_PIN0) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOG_PIN1) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOG_PIN2) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOG_PIN3) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOG_PIN4) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOG_PIN5) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOG_PIN6) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOG_PIN7) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOG_PIN8) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOG_PIN9) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOG_PIN10) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOG_PIN11) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOG_PIN12) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOG_PIN13) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOG_PIN14) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOG_PIN15))

#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_HIGH(GPIOG_PIN0) |     \
                                     PIN_OSPEED_HIGH(GPIOG_PIN1) |     \
                                     PIN_OSPEED_HIGH(GPIOG_PIN2) |     \
                                     PIN_OSPEED_HIGH(GPIOG_PIN3) |     \
                                     PIN_OSPEED_HIGH(GPIOG_PIN4) |     \
                                     PIN_OSPEED_HIGH(GPIOG_PIN5) |     \
                                     PIN_OSPEED_HIGH(GPIOG_PIN6) |     \
                                     PIN_OSPEED_HIGH(GPIOG_PIN7) |     \
                                     PIN_OSPEED_HIGH(GPIOG_PIN8) |     \
                                     PIN_OSPEED_HIGH(GPIOG_PIN9) |     \
                                     PIN_OSPEED_HIGH(GPIOG_PIN10) |    \
                                     PIN_OSPEED_HIGH(GPIOG_PIN11) |    \
                                     PIN_OSPEED_HIGH(GPIOG_PIN12) |    \
                                     PIN_OSPEED_HIGH(GPIOG_PIN13) |    \
                                     PIN_OSPEED_HIGH(GPIOG_PIN14) |    \
                                     PIN_OSPEED_HIGH(GPIOG_PIN15))

#define VAL_GPIOG_PUPDR             (PIN_PUPDR_PULLUP(GPIOG_PIN0) |    \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN1) |    \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN2) |    \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN3) |    \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN4) |    \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN5) |    \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN6) |    \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN7) |    \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN8) |    \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN9) |    \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN10) |   \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN11) |   \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN12) |   \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN13) |   \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN14) |   \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN15))

#define VAL_GPIOG_ODR               (PIN_ODR_HIGH(GPIOG_PIN0) |        \
                                     PIN_ODR_HIGH(GPIOG_PIN1) |        \
                                     PIN_ODR_HIGH(GPIOG_PIN2) |        \
                                     PIN_ODR_HIGH(GPIOG_PIN3) |        \
                                     PIN_ODR_HIGH(GPIOG_PIN4) |        \
                                     PIN_ODR_HIGH(GPIOG_PIN5) |        \
                                     PIN_ODR_HIGH(GPIOG_PIN6) |        \
                                     PIN_ODR_HIGH(GPIOG_PIN7) |        \
                                     PIN_ODR_HIGH(GPIOG_PIN8) |        \
                                     PIN_ODR_HIGH(GPIOG_PIN9) |        \
                                     PIN_ODR_HIGH(GPIOG_PIN10) |       \
                                     PIN_ODR_HIGH(GPIOG_PIN11) |       \
                                     PIN_ODR_HIGH(GPIOG_PIN12) |       \
                                     PIN_ODR_HIGH(GPIOG_PIN13) |       \
                                     PIN_ODR_HIGH(GPIOG_PIN14) |       \
                                     PIN_ODR_HIGH(GPIOG_PIN15))

#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(GPIOG_PIN0, 0U) |     \
                                     PIN_AFIO_AF(GPIOG_PIN1, 0U) |     \
                                     PIN_AFIO_AF(GPIOG_PIN2, 0U) |     \
                                     PIN_AFIO_AF(GPIOG_PIN3, 0U) |     \
                                     PIN_AFIO_AF(GPIOG_PIN4, 0U) |     \
                                     PIN_AFIO_AF(GPIOG_PIN5, 0U) |     \
                                     PIN_AFIO_AF(GPIOG_PIN6, 0U) |     \
                                     PIN_AFIO_AF(GPIOG_PIN7, 0U))
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIOG_PIN8, 0U) |     \
                                     PIN_AFIO_AF(GPIOG_PIN9, 0U) |     \
                                     PIN_AFIO_AF(GPIOG_PIN10, 0U) |    \
                                     PIN_AFIO_AF(GPIOG_PIN11, 0U) |    \
                                     PIN_AFIO_AF(GPIOG_PIN12, 0U) |    \
                                     PIN_AFIO_AF(GPIOG_PIN13, 0U) |    \
                                     PIN_AFIO_AF(GPIOG_PIN14, 0U) |    \
                                     PIN_AFIO_AF(GPIOG_PIN15, 0U))

/*
* GPIOH setup:
*   
* PH0  - PIN0                      (input pullup).
* PH1  - PIN1                      (input pullup).
* PH2  - PIN2                      (input pullup).
* PH3  - PIN3                      (input pullup).
* PH4  - PIN4                      (input pullup).
* PH5  - PIN5                      (input pullup).
* PH6  - PIN6                      (input pullup).
* PH7  - PIN7                      (input pullup).
* PH8  - PIN8                      (input pullup).
* PH9  - PIN9                      (input pullup).
* PH10 - PIN10                     (input pullup).
* PH11 - PIN11                     (input pullup).
* PH12 - PIN12                     (input pullup).
* PH13 - PIN13                     (input pullup).
* PH14 - PIN14                     (input pullup).
* PH15 - PIN15                     (input pullup).
*/

#define VAL_GPIOH_MODER             (PIN_MODE_ANALOG(GPIOH_PIN0) |      \
                                     PIN_MODE_ANALOG(GPIOH_PIN1) |      \
                                     PIN_MODE_ANALOG(GPIOH_PIN2) |      \
                                     PIN_MODE_ANALOG(GPIOH_PIN3) |      \
                                     PIN_MODE_ANALOG(GPIOH_PIN4) |      \
                                     PIN_MODE_ANALOG(GPIOH_PIN5) |      \
                                     PIN_MODE_ANALOG(GPIOH_PIN6) |      \
                                     PIN_MODE_ANALOG(GPIOH_PIN7) |      \
                                     PIN_MODE_ANALOG(GPIOH_PIN8) |      \
                                     PIN_MODE_ANALOG(GPIOH_PIN9) |      \
                                     PIN_MODE_ANALOG(GPIOH_PIN10) |     \
                                     PIN_MODE_ANALOG(GPIOH_PIN11) |     \
                                     PIN_MODE_ANALOG(GPIOH_PIN12) |     \
                                     PIN_MODE_ANALOG(GPIOH_PIN13) |     \
                                     PIN_MODE_ANALOG(GPIOH_PIN14) |     \
                                     PIN_MODE_ANALOG(GPIOH_PIN15))

#define VAL_GPIOH_OTYPER            (PIN_OTYPE_OPENDRAIN(GPIOH_PIN0) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_PIN1) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_PIN2) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_PIN3) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_PIN4) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_PIN5) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_PIN6) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_PIN7) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_PIN8) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_PIN9) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_PIN10) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_PIN11) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_PIN12) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_PIN13) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_PIN14) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_PIN15))

#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_HIGH(GPIOH_PIN0) |     \
                                     PIN_OSPEED_HIGH(GPIOH_PIN1) |     \
                                     PIN_OSPEED_HIGH(GPIOH_PIN2) |     \
                                     PIN_OSPEED_HIGH(GPIOH_PIN3) |     \
                                     PIN_OSPEED_HIGH(GPIOH_PIN4) |     \
                                     PIN_OSPEED_HIGH(GPIOH_PIN5) |     \
                                     PIN_OSPEED_HIGH(GPIOH_PIN6) |     \
                                     PIN_OSPEED_HIGH(GPIOH_PIN7) |     \
                                     PIN_OSPEED_HIGH(GPIOH_PIN8) |     \
                                     PIN_OSPEED_HIGH(GPIOH_PIN9) |     \
                                     PIN_OSPEED_HIGH(GPIOH_PIN10) |    \
                                     PIN_OSPEED_HIGH(GPIOH_PIN11) |    \
                                     PIN_OSPEED_HIGH(GPIOH_PIN12) |    \
                                     PIN_OSPEED_HIGH(GPIOH_PIN13) |    \
                                     PIN_OSPEED_HIGH(GPIOH_PIN14) |    \
                                     PIN_OSPEED_HIGH(GPIOH_PIN15))

#define VAL_GPIOH_PUPDR             (PIN_PUPDR_PULLUP(GPIOH_PIN0) |    \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN1) |    \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN2) |    \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN3) |    \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN4) |    \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN5) |    \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN6) |    \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN7) |    \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN8) |    \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN9) |    \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN10) |   \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN11) |   \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN12) |   \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN13) |   \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN14) |   \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN15))

#define VAL_GPIOH_ODR               (PIN_ODR_HIGH(GPIOH_PIN0) |        \
                                     PIN_ODR_HIGH(GPIOH_PIN1) |        \
                                     PIN_ODR_HIGH(GPIOH_PIN2) |        \
                                     PIN_ODR_HIGH(GPIOH_PIN3) |        \
                                     PIN_ODR_HIGH(GPIOH_PIN4) |        \
                                     PIN_ODR_HIGH(GPIOH_PIN5) |        \
                                     PIN_ODR_HIGH(GPIOH_PIN6) |        \
                                     PIN_ODR_HIGH(GPIOH_PIN7) |        \
                                     PIN_ODR_HIGH(GPIOH_PIN8) |        \
                                     PIN_ODR_HIGH(GPIOH_PIN9) |        \
                                     PIN_ODR_HIGH(GPIOH_PIN10) |       \
                                     PIN_ODR_HIGH(GPIOH_PIN11) |       \
                                     PIN_ODR_HIGH(GPIOH_PIN12) |       \
                                     PIN_ODR_HIGH(GPIOH_PIN13) |       \
                                     PIN_ODR_HIGH(GPIOH_PIN14) |       \
                                     PIN_ODR_HIGH(GPIOH_PIN15))

#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_PIN0, 0U) |     \
                                     PIN_AFIO_AF(GPIOH_PIN1, 0U) |     \
                                     PIN_AFIO_AF(GPIOH_PIN2, 0U) |     \
                                     PIN_AFIO_AF(GPIOH_PIN3, 0U) |     \
                                     PIN_AFIO_AF(GPIOH_PIN4, 0U) |     \
                                     PIN_AFIO_AF(GPIOH_PIN5, 0U) |     \
                                     PIN_AFIO_AF(GPIOH_PIN6, 0U) |     \
                                     PIN_AFIO_AF(GPIOH_PIN7, 0U))
#define VAL_GPIOH_AFRH              (PIN_AFIO_AF(GPIOH_PIN8, 0U) |     \
                                     PIN_AFIO_AF(GPIOH_PIN9, 0U) |     \
                                     PIN_AFIO_AF(GPIOH_PIN10, 0U) |    \
                                     PIN_AFIO_AF(GPIOH_PIN11, 0U) |    \
                                     PIN_AFIO_AF(GPIOH_PIN12, 0U) |    \
                                     PIN_AFIO_AF(GPIOH_PIN13, 0U) |    \
                                     PIN_AFIO_AF(GPIOH_PIN14, 0U) |    \
                                     PIN_AFIO_AF(GPIOH_PIN15, 0U))

/*
* GPIOI setup:
*   
* PI0  - PIN0                      (input pullup).
* PI1  - PIN1                      (input pullup).
* PI2  - PIN2                      (input pullup).
* PI3  - PIN3                      (input pullup).
* PI4  - PIN4                      (input pullup).
* PI5  - PIN5                      (input pullup).
* PI6  - PIN6                      (input pullup).
* PI7  - PIN7                      (input pullup).
* PI8  - PIN8                      (input pullup).
* PI9  - PIN9                      (input pullup).
* PI10 - PIN10                     (input pullup).
* PI11 - PIN11                     (input pullup).
* PI12 - PIN12                     (input pullup).
* PI13 - PIN13                     (input pullup).
* PI14 - PIN14                     (input pullup).
* PI15 - PIN15                     (input pullup).
*/

#define VAL_GPIOI_MODER             (PIN_MODE_ANALOG(GPIOI_PIN0) |      \
                                     PIN_MODE_ANALOG(GPIOI_PIN1) |      \
                                     PIN_MODE_ANALOG(GPIOI_PIN2) |      \
                                     PIN_MODE_ANALOG(GPIOI_PIN3) |      \
                                     PIN_MODE_ANALOG(GPIOI_PIN4) |      \
                                     PIN_MODE_ANALOG(GPIOI_PIN5) |      \
                                     PIN_MODE_ANALOG(GPIOI_PIN6) |      \
                                     PIN_MODE_ANALOG(GPIOI_PIN7) |      \
                                     PIN_MODE_ANALOG(GPIOI_PIN8) |      \
                                     PIN_MODE_ANALOG(GPIOI_PIN9) |      \
                                     PIN_MODE_ANALOG(GPIOI_PIN10) |     \
                                     PIN_MODE_ANALOG(GPIOI_PIN11) |     \
                                     PIN_MODE_ANALOG(GPIOI_PIN12) |     \
                                     PIN_MODE_ANALOG(GPIOI_PIN13) |     \
                                     PIN_MODE_ANALOG(GPIOI_PIN14) |     \
                                     PIN_MODE_ANALOG(GPIOI_PIN15))

#define VAL_GPIOI_OTYPER            (PIN_OTYPE_OPENDRAIN(GPIOI_PIN0) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOI_PIN1) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOI_PIN2) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOI_PIN3) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOI_PIN4) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOI_PIN5) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOI_PIN6) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOI_PIN7) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOI_PIN8) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOI_PIN9) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOI_PIN10) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOI_PIN11) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOI_PIN12) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOI_PIN13) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOI_PIN14) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOI_PIN15))

#define VAL_GPIOI_OSPEEDR           (PIN_OSPEED_HIGH(GPIOI_PIN0) |     \
                                     PIN_OSPEED_HIGH(GPIOI_PIN1) |     \
                                     PIN_OSPEED_HIGH(GPIOI_PIN2) |     \
                                     PIN_OSPEED_HIGH(GPIOI_PIN3) |     \
                                     PIN_OSPEED_HIGH(GPIOI_PIN4) |     \
                                     PIN_OSPEED_HIGH(GPIOI_PIN5) |     \
                                     PIN_OSPEED_HIGH(GPIOI_PIN6) |     \
                                     PIN_OSPEED_HIGH(GPIOI_PIN7) |     \
                                     PIN_OSPEED_HIGH(GPIOI_PIN8) |     \
                                     PIN_OSPEED_HIGH(GPIOI_PIN9) |     \
                                     PIN_OSPEED_HIGH(GPIOI_PIN10) |    \
                                     PIN_OSPEED_HIGH(GPIOI_PIN11) |    \
                                     PIN_OSPEED_HIGH(GPIOI_PIN12) |    \
                                     PIN_OSPEED_HIGH(GPIOI_PIN13) |    \
                                     PIN_OSPEED_HIGH(GPIOI_PIN14) |    \
                                     PIN_OSPEED_HIGH(GPIOI_PIN15))

#define VAL_GPIOI_PUPDR             (PIN_PUPDR_PULLUP(GPIOI_PIN0) |    \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN1) |    \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN2) |    \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN3) |    \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN4) |    \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN5) |    \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN6) |    \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN7) |    \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN8) |    \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN9) |    \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN10) |   \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN11) |   \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN12) |   \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN13) |   \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN14) |   \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN15))

#define VAL_GPIOI_ODR               (PIN_ODR_HIGH(GPIOI_PIN0) |        \
                                     PIN_ODR_HIGH(GPIOI_PIN1) |        \
                                     PIN_ODR_HIGH(GPIOI_PIN2) |        \
                                     PIN_ODR_HIGH(GPIOI_PIN3) |        \
                                     PIN_ODR_HIGH(GPIOI_PIN4) |        \
                                     PIN_ODR_HIGH(GPIOI_PIN5) |        \
                                     PIN_ODR_HIGH(GPIOI_PIN6) |        \
                                     PIN_ODR_HIGH(GPIOI_PIN7) |        \
                                     PIN_ODR_HIGH(GPIOI_PIN8) |        \
                                     PIN_ODR_HIGH(GPIOI_PIN9) |        \
                                     PIN_ODR_HIGH(GPIOI_PIN10) |       \
                                     PIN_ODR_HIGH(GPIOI_PIN11) |       \
                                     PIN_ODR_HIGH(GPIOI_PIN12) |       \
                                     PIN_ODR_HIGH(GPIOI_PIN13) |       \
                                     PIN_ODR_HIGH(GPIOI_PIN14) |       \
                                     PIN_ODR_HIGH(GPIOI_PIN15))

#define VAL_GPIOI_AFRL              (PIN_AFIO_AF(GPIOI_PIN0, 0U) |     \
                                     PIN_AFIO_AF(GPIOI_PIN1, 0U) |     \
                                     PIN_AFIO_AF(GPIOI_PIN2, 0U) |     \
                                     PIN_AFIO_AF(GPIOI_PIN3, 0U) |     \
                                     PIN_AFIO_AF(GPIOI_PIN4, 0U) |     \
                                     PIN_AFIO_AF(GPIOI_PIN5, 0U) |     \
                                     PIN_AFIO_AF(GPIOI_PIN6, 0U) |     \
                                     PIN_AFIO_AF(GPIOI_PIN7, 0U))
#define VAL_GPIOI_AFRH              (PIN_AFIO_AF(GPIOI_PIN8, 0U) |     \
                                     PIN_AFIO_AF(GPIOI_PIN9, 0U) |     \
                                     PIN_AFIO_AF(GPIOI_PIN10, 0U) |    \
                                     PIN_AFIO_AF(GPIOI_PIN11, 0U) |    \
                                     PIN_AFIO_AF(GPIOI_PIN12, 0U) |    \
                                     PIN_AFIO_AF(GPIOI_PIN13, 0U) |    \
                                     PIN_AFIO_AF(GPIOI_PIN14, 0U) |    \
                                     PIN_AFIO_AF(GPIOI_PIN15, 0U))
/*
* GPIOJ setup:
*   
* PJ0  - PIN0                      (input pullup).
* PJ1  - PIN1                      (input pullup).
* PJ2  - PIN2                      (input pullup).
* PJ3  - PIN3                      (input pullup).
* PJ4  - PIN4                      (input pullup).
* PJ5  - PIN5                      (input pullup).
* PJ6  - PIN6                      (input pullup).
* PJ7  - PIN7                      (input pullup).
* PJ8  - PIN8                      (input pullup).
* PJ9  - PIN9                      (input pullup).
* PJ10 - PIN10                     (input pullup).
* PJ11 - PIN11                     (input pullup).
* PJ12 - PIN12                     (input pullup).
* PJ13 - PIN13                     (input pullup).
* PJ14 - PIN14                     (input pullup).
* PJ15 - PIN15                     (input pullup).
*/

#define VAL_GPIOJ_MODER             (PIN_MODE_ANALOG(GPIOJ_PIN0) |      \
                                     PIN_MODE_ANALOG(GPIOJ_PIN1) |      \
                                     PIN_MODE_ANALOG(GPIOJ_PIN2) |      \
                                     PIN_MODE_ANALOG(GPIOJ_PIN3) |      \
                                     PIN_MODE_ANALOG(GPIOJ_PIN4) |      \
                                     PIN_MODE_ANALOG(GPIOJ_PIN5) |      \
                                     PIN_MODE_ANALOG(GPIOJ_PIN6) |      \
                                     PIN_MODE_ANALOG(GPIOJ_PIN7) |      \
                                     PIN_MODE_ANALOG(GPIOJ_PIN8) |      \
                                     PIN_MODE_ANALOG(GPIOJ_PIN9) |      \
                                     PIN_MODE_ANALOG(GPIOJ_PIN10) |     \
                                     PIN_MODE_ANALOG(GPIOJ_PIN11) |     \
                                     PIN_MODE_ANALOG(GPIOJ_PIN12) |     \
                                     PIN_MODE_ANALOG(GPIOJ_PIN13) |     \
                                     PIN_MODE_ANALOG(GPIOJ_PIN14) |     \
                                     PIN_MODE_ANALOG(GPIOJ_PIN15))

#define VAL_GPIOJ_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOJ_PIN0) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN1) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN2) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN3) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN4) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN5) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN6) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN7) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN8) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN9) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN10) | \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN11) | \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN12) | \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN13) | \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN14) | \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN15))

#define VAL_GPIOJ_OSPEEDR           (PIN_OSPEED_HIGH(GPIOJ_PIN0) |     \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN1) |     \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN2) |     \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN3) |     \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN4) |     \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN5) |     \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN6) |     \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN7) |     \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN8) |     \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN9) |     \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN10) |    \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN11) |    \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN12) |    \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN13) |    \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN14) |    \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN15))

#define VAL_GPIOJ_PUPDR             (PIN_PUPDR_PULLUP(GPIOJ_PIN0) |    \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN1) |    \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN2) |    \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN3) |    \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN4) |    \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN5) |    \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN6) |    \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN7) |    \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN8) |    \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN9) |    \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN10) |   \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN11) |   \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN12) |   \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN13) |   \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN14) |   \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN15))

#define VAL_GPIOJ_ODR               (PIN_ODR_HIGH(GPIOJ_PIN0) |        \
                                     PIN_ODR_HIGH(GPIOJ_PIN1) |        \
                                     PIN_ODR_HIGH(GPIOJ_PIN2) |        \
                                     PIN_ODR_HIGH(GPIOJ_PIN3) |        \
                                     PIN_ODR_HIGH(GPIOJ_PIN4) |        \
                                     PIN_ODR_HIGH(GPIOJ_PIN5) |        \
                                     PIN_ODR_HIGH(GPIOJ_PIN6) |        \
                                     PIN_ODR_HIGH(GPIOJ_PIN7) |        \
                                     PIN_ODR_HIGH(GPIOJ_PIN8) |        \
                                     PIN_ODR_HIGH(GPIOJ_PIN9) |        \
                                     PIN_ODR_HIGH(GPIOJ_PIN10) |       \
                                     PIN_ODR_HIGH(GPIOJ_PIN11) |       \
                                     PIN_ODR_HIGH(GPIOJ_PIN12) |       \
                                     PIN_ODR_HIGH(GPIOJ_PIN13) |       \
                                     PIN_ODR_HIGH(GPIOJ_PIN14) |       \
                                     PIN_ODR_HIGH(GPIOJ_PIN15))

#define VAL_GPIOJ_AFRL              (PIN_AFIO_AF(GPIOJ_PIN0, 0U) |     \
                                     PIN_AFIO_AF(GPIOJ_PIN1, 0U) |     \
                                     PIN_AFIO_AF(GPIOJ_PIN2, 0U) |     \
                                     PIN_AFIO_AF(GPIOJ_PIN3, 0U) |     \
                                     PIN_AFIO_AF(GPIOJ_PIN4, 0U) |     \
                                     PIN_AFIO_AF(GPIOJ_PIN5, 0U) |     \
                                     PIN_AFIO_AF(GPIOJ_PIN6, 0U) |     \
                                     PIN_AFIO_AF(GPIOJ_PIN7, 0U))
#define VAL_GPIOJ_AFRH              (PIN_AFIO_AF(GPIOJ_PIN8, 0U) |     \
                                     PIN_AFIO_AF(GPIOJ_PIN9, 0U) |     \
                                     PIN_AFIO_AF(GPIOJ_PIN10, 0U) |    \
                                     PIN_AFIO_AF(GPIOJ_PIN11, 0U) |    \
                                     PIN_AFIO_AF(GPIOJ_PIN12, 0U) |    \
                                     PIN_AFIO_AF(GPIOJ_PIN13, 0U) |    \
                                     PIN_AFIO_AF(GPIOJ_PIN14, 0U) |    \
                                     PIN_AFIO_AF(GPIOJ_PIN15, 0U))

/*
* GPIOK setup:
*   
* PK0  - PIN0                      (input pullup).
* PK1  - PIN1                      (input pullup).
* PK2  - PIN2                      (input pullup).
* PK3  - PIN3                      (input pullup).
* PK4  - PIN4                      (input pullup).
* PK5  - PIN5                      (input pullup).
* PK6  - PIN6                      (input pullup).
* PK7  - PIN7                      (input pullup).
* PK8  - PIN8                      (input pullup).
* PK9  - PIN9                      (input pullup).
* PK10 - PIN10                     (input pullup).
* PK11 - PIN11                     (input pullup).
* PK12 - PIN12                     (input pullup).
* PK13 - PIN13                     (input pullup).
* PK14 - PIN14                     (input pullup).
* PK15 - PIN15                     (input pullup).
*/

#define VAL_GPIOK_MODER             (PIN_MODE_ANALOG(GPIOK_PIN0) |      \
                                     PIN_MODE_ANALOG(GPIOK_PIN1) |      \
                                     PIN_MODE_ANALOG(GPIOK_PIN2) |      \
                                     PIN_MODE_ANALOG(GPIOK_PIN3) |      \
                                     PIN_MODE_ANALOG(GPIOK_PIN4) |      \
                                     PIN_MODE_ANALOG(GPIOK_PIN5) |      \
                                     PIN_MODE_ANALOG(GPIOK_PIN6) |      \
                                     PIN_MODE_ANALOG(GPIOK_PIN7) |      \
                                     PIN_MODE_ANALOG(GPIOK_PIN8) |      \
                                     PIN_MODE_ANALOG(GPIOK_PIN9) |      \
                                     PIN_MODE_ANALOG(GPIOK_PIN10) |     \
                                     PIN_MODE_ANALOG(GPIOK_PIN11) |     \
                                     PIN_MODE_ANALOG(GPIOK_PIN12) |     \
                                     PIN_MODE_ANALOG(GPIOK_PIN13) |     \
                                     PIN_MODE_ANALOG(GPIOK_PIN14) |     \
                                     PIN_MODE_ANALOG(GPIOK_PIN15))

#define VAL_GPIOK_OTYPER            (PIN_OTYPE_OPENDRAIN(GPIOK_PIN0) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOK_PIN1) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOK_PIN2) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOK_PIN3) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOK_PIN4) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOK_PIN5) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOK_PIN6) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOK_PIN7) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOK_PIN8) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOK_PIN9) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOK_PIN10) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOK_PIN11) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOK_PIN12) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOK_PIN13) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOK_PIN14) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOK_PIN15))

#define VAL_GPIOK_OSPEEDR           (PIN_OSPEED_HIGH(GPIOK_PIN0) |     \
                                     PIN_OSPEED_HIGH(GPIOK_PIN1) |     \
                                     PIN_OSPEED_HIGH(GPIOK_PIN2) |     \
                                     PIN_OSPEED_HIGH(GPIOK_PIN3) |     \
                                     PIN_OSPEED_HIGH(GPIOK_PIN4) |     \
                                     PIN_OSPEED_HIGH(GPIOK_PIN5) |     \
                                     PIN_OSPEED_HIGH(GPIOK_PIN6) |     \
                                     PIN_OSPEED_HIGH(GPIOK_PIN7) |     \
                                     PIN_OSPEED_HIGH(GPIOK_PIN8) |     \
                                     PIN_OSPEED_HIGH(GPIOK_PIN9) |     \
                                     PIN_OSPEED_HIGH(GPIOK_PIN10) |    \
                                     PIN_OSPEED_HIGH(GPIOK_PIN11) |    \
                                     PIN_OSPEED_HIGH(GPIOK_PIN12) |    \
                                     PIN_OSPEED_HIGH(GPIOK_PIN13) |    \
                                     PIN_OSPEED_HIGH(GPIOK_PIN14) |    \
                                     PIN_OSPEED_HIGH(GPIOK_PIN15))

#define VAL_GPIOK_PUPDR             (PIN_PUPDR_PULLUP(GPIOK_PIN0) |    \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN1) |    \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN2) |    \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN3) |    \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN4) |    \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN5) |    \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN6) |    \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN7) |    \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN8) |    \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN9) |    \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN10) |   \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN11) |   \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN12) |   \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN13) |   \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN14) |   \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN15))

#define VAL_GPIOK_ODR               (PIN_ODR_HIGH(GPIOK_PIN0) |        \
                                     PIN_ODR_HIGH(GPIOK_PIN1) |        \
                                     PIN_ODR_HIGH(GPIOK_PIN2) |        \
                                     PIN_ODR_HIGH(GPIOK_PIN3) |        \
                                     PIN_ODR_HIGH(GPIOK_PIN4) |        \
                                     PIN_ODR_HIGH(GPIOK_PIN5) |        \
                                     PIN_ODR_HIGH(GPIOK_PIN6) |        \
                                     PIN_ODR_HIGH(GPIOK_PIN7) |        \
                                     PIN_ODR_HIGH(GPIOK_PIN8) |        \
                                     PIN_ODR_HIGH(GPIOK_PIN9) |        \
                                     PIN_ODR_HIGH(GPIOK_PIN10) |       \
                                     PIN_ODR_HIGH(GPIOK_PIN11) |       \
                                     PIN_ODR_HIGH(GPIOK_PIN12) |       \
                                     PIN_ODR_HIGH(GPIOK_PIN13) |       \
                                     PIN_ODR_HIGH(GPIOK_PIN14) |       \
                                     PIN_ODR_HIGH(GPIOK_PIN15))

#define VAL_GPIOK_AFRL              (PIN_AFIO_AF(GPIOK_PIN0, 0U) |     \
                                     PIN_AFIO_AF(GPIOK_PIN1, 0U) |     \
                                     PIN_AFIO_AF(GPIOK_PIN2, 0U) |     \
                                     PIN_AFIO_AF(GPIOK_PIN3, 0U) |     \
                                     PIN_AFIO_AF(GPIOK_PIN4, 0U) |     \
                                     PIN_AFIO_AF(GPIOK_PIN5, 0U) |     \
                                     PIN_AFIO_AF(GPIOK_PIN6, 0U) |     \
                                     PIN_AFIO_AF(GPIOK_PIN7, 0U))
#define VAL_GPIOK_AFRH              (PIN_AFIO_AF(GPIOK_PIN8, 0U) |     \
                                     PIN_AFIO_AF(GPIOK_PIN9, 0U) |     \
                                     PIN_AFIO_AF(GPIOK_PIN10, 0U) |    \
                                     PIN_AFIO_AF(GPIOK_PIN11, 0U) |    \
                                     PIN_AFIO_AF(GPIOK_PIN12, 0U) |    \
                                     PIN_AFIO_AF(GPIOK_PIN13, 0U) |    \
                                     PIN_AFIO_AF(GPIOK_PIN14, 0U) |    \
                                     PIN_AFIO_AF(GPIOK_PIN15, 0U))

// clang-format on

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C"
{
#endif
    void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* BOARD_H */
