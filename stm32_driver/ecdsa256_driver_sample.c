//
// simple driver to test "ecdsa256" core in hardware
//

//
// note, that the test program needs a custom bitstream where
// the core is located at offset 0 (without the core selector)
//

// stm32 headers
#include "stm-init.h"
#include "stm-led.h"
#include "stm-fmc.h"

// locations of core registers
#define CORE_ADDR_NAME0			(0x00 << 2)
#define CORE_ADDR_NAME1			(0x01 << 2)
#define CORE_ADDR_VERSION		(0x02 << 2)
#define CORE_ADDR_CONTROL		(0x08 << 2)
#define CORE_ADDR_STATUS		(0x09 << 2)

// locations of data buffers
#define CORE_ADDR_BUF_K			(0x20 << 2)
#define CORE_ADDR_BUF_X			(0x28 << 2)
#define CORE_ADDR_BUF_Y			(0x30 << 2)

// bit maps
#define CORE_CONTROL_BIT_NEXT		0x00000002
#define CORE_STATUS_BIT_READY		0x00000002

// curve selection
#define USE_CURVE			1

#include "ecdsa_test_vector_nsa.h"
#include "ecdsa_test_vector_randomized.h"

#define bool uint32_t // very dirty hack, but works in this particular case
#include "ecdsa_fpga_lowlevel.h"
#include "ecdsa_fpga_multiword.h"
#include "ecdsa_fpga_curve.h"
#undef bool

#define BUF_NUM_WORDS FPGA_OPERAND_NUM_WORDS

//
// test vectors
//
static const uint32_t p256_d_nsa[BUF_NUM_WORDS]  = ECDSA_P256_D_NSA_INIT;
static const uint32_t p256_qx_nsa[BUF_NUM_WORDS] = ECDSA_P256_QX_NSA_INIT;
static const uint32_t p256_qy_nsa[BUF_NUM_WORDS] = ECDSA_P256_QY_NSA_INIT;

static const uint32_t p256_k_nsa[BUF_NUM_WORDS]  = ECDSA_P256_K_NSA_INIT;
static const uint32_t p256_rx_nsa[BUF_NUM_WORDS] = ECDSA_P256_RX_NSA_INIT;
static const uint32_t p256_ry_nsa[BUF_NUM_WORDS] = ECDSA_P256_RY_NSA_INIT;

static const uint32_t p256_d_random[BUF_NUM_WORDS]  = ECDSA_P256_D_RANDOM_INIT;
static const uint32_t p256_qx_random[BUF_NUM_WORDS] = ECDSA_P256_QX_RANDOM_INIT;
static const uint32_t p256_qy_random[BUF_NUM_WORDS] = ECDSA_P256_QY_RANDOM_INIT;


static const uint32_t p256_gx[BUF_NUM_WORDS] = ECDSA_P256_GX_INIT;
static const uint32_t p256_gy[BUF_NUM_WORDS] = ECDSA_P256_GY_INIT;
static const uint32_t p256_hx[BUF_NUM_WORDS] = ECDSA_P256_HX_INIT;
static const uint32_t p256_hy[BUF_NUM_WORDS] = ECDSA_P256_HY_INIT;
static const uint32_t p256_n[BUF_NUM_WORDS]  = ECDSA_P256_N_INIT;

static uint32_t p256_zero[BUF_NUM_WORDS];
static uint32_t p256_two [BUF_NUM_WORDS];
static uint32_t p256_n1  [BUF_NUM_WORDS];
static uint32_t p256_n2  [BUF_NUM_WORDS];


//
// prototypes
//
void toggle_yellow_led(void);
int test_p256_multiplier(const uint32_t *k, const uint32_t *px, const uint32_t *py);

//
// test routine
//
int main()
{
  int ok;

  stm_init();
  fmc_init();

  led_on(LED_GREEN);
  led_off(LED_RED);

  led_off(LED_YELLOW);
  led_off(LED_BLUE);

  uint32_t core_name0;
  uint32_t core_name1;
	uint32_t core_version;

  fmc_read_32(CORE_ADDR_NAME0,   &core_name0);
  fmc_read_32(CORE_ADDR_NAME1,   &core_name1);
	fmc_read_32(CORE_ADDR_VERSION, &core_version);

  // "ecds", "a256"
  if ((core_name0 != 0x65636473) || (core_name1 != 0x61323536)) {
    led_off(LED_GREEN);
    led_on(LED_RED);
    while (1);
  }

	// prepare more numbers
	size_t w;
	for (w=0; w<BUF_NUM_WORDS; w++)
	{	p256_zero[w] = 0;
		p256_two [w] = 0;
		p256_n1  [w] = p256_n   [w];
		p256_n2  [w] = p256_n   [w];
	}
	
	p256_two[BUF_NUM_WORDS-1] += 2;
	p256_n1 [BUF_NUM_WORDS-1] += 1;
	p256_n2 [BUF_NUM_WORDS-1] += 2;
	
	
  // repeat forever
  while (1)
    {
      ok = 1;
			
      ok = ok && test_p256_multiplier(p256_d_nsa,    p256_qx_nsa,    p256_qy_nsa);	  /* Q = d * G */
      ok = ok && test_p256_multiplier(p256_k_nsa,    p256_rx_nsa,    p256_ry_nsa);	  /* R = k * G */
			ok = ok && test_p256_multiplier(p256_d_random, p256_qx_random, p256_qy_random);	/* Q = d * G */
			
      ok = ok && test_p256_multiplier(p256_n,   p256_zero, p256_zero); /* O = n * G */
      ok = ok && test_p256_multiplier(p256_n1,  p256_gx,   p256_gy);	 /* G = (n + 1) * G */
			
	//
	// The following two vectors test the virtually never taken path in the curve point
	// addition routine when both input points are the same. During the first test (2 * G)
	// the double of the base point is computed at the second doubling step of the multiplication
	// algorithm, which does not require any special handling. During the second test the
	// precomputed double of the base point (stored in internal read-only memory) is returned,
	// because after doubling of G * ((n + 1) / 2) we get G * (n + 1) = G. The adder then has to
	// compute G + G for which the formulae don't work, and special handling is required. The two
	// test vectors verify that the hardcoded double of the base point matches the one computed
	// on the fly. Note that in practice one should never be multiplying by anything larger than (n-1),
	// because both the secret key and the per-message (random) number must be from [1, n-1].
	//
			ok = ok && test_p256_multiplier(p256_two, p256_hx,   p256_hy);	 /* H = 2 * G */
      ok = ok && test_p256_multiplier(p256_n2,  p256_hx,   p256_hy);	 /* H = (n + 2) * G */

			
      if (!ok) {
				led_off(LED_GREEN);
				led_on(LED_RED);
			}

      toggle_yellow_led();
    }
}


//
// this routine uses the hardware multiplier to obtain Q(qx,qy), which is the
// scalar multiple of the base point, qx and qy are then compared to the values
// px and py (correct result known in advance)
//
int test_p256_multiplier(const uint32_t *k, const uint32_t *px, const uint32_t *py)
{
  int i, num_cyc;
  uint32_t reg_control, reg_status;
  uint32_t k_word, qx_word, qy_word;

  // fill k
  for (i=0; i<BUF_NUM_WORDS; i++) {
    k_word = k[i];
    fmc_write_32(CORE_ADDR_BUF_K + ((BUF_NUM_WORDS - (i + 1)) * sizeof(uint32_t)), k_word);
  }

  // clear 'next' control bit, then set 'next' control bit again to trigger new operation
  reg_control = 0;
  fmc_write_32(CORE_ADDR_CONTROL, reg_control);
  reg_control = CORE_CONTROL_BIT_NEXT;
  fmc_write_32(CORE_ADDR_CONTROL, reg_control);

  // wait for 'ready' status bit to be set
  num_cyc = 0;
  do {
    num_cyc++;
    fmc_read_32(CORE_ADDR_STATUS, &reg_status);
  }
  while (!(reg_status & CORE_STATUS_BIT_READY));

  // read back x and y word-by-word, then compare to the reference values
  for (i=0; i<BUF_NUM_WORDS; i++) {
    fmc_read_32(CORE_ADDR_BUF_X + (i * sizeof(uint32_t)), &qx_word);
    fmc_read_32(CORE_ADDR_BUF_Y + (i * sizeof(uint32_t)), &qy_word);

    if ((qx_word != px[BUF_NUM_WORDS - (i + 1)])) return 0;
    if ((qy_word != py[BUF_NUM_WORDS - (i + 1)])) return 0;
  }

  // everything went just fine
  return 1;
}

//
// toggle the yellow led to indicate that we're not stuck somewhere
//
void toggle_yellow_led(void)
{
  static int led_state = 0;

  led_state = !led_state;

  if (led_state) led_on(LED_YELLOW);
  else           led_off(LED_YELLOW);
}


//
// systick
//
void SysTick_Handler(void)
{
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}


//
// end of file
//
