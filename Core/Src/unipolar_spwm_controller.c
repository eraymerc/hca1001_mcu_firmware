/*
 * unipolar_spwm_controller.c
 *
 * Created on: 2026
 * Author: User
 */

#include "main.h" // TIM_TypeDef ve uint32_t tanimlamalari icin gereklidir
#include "unipolar_spwm_controller.h"

/**
 * @brief Unipolar SPWM Duty Cycle Guncelleme Fonksiyonu
 * * Bu fonksiyon, disaridan gelen anlik sinyal genligine (amplitude) gore
 * ilgili Timer'in Channel 1 ve Channel 2 karsilastirma (CCR) degerlerini ayarlar.
 * Unipolar SPWM teknigi geregi, CH2'ye CH1'in 180 derece faz farkli (tersi) degeri yuklenir.
 * * @param tim: Kullanilacak Timer'in adresi (Orn: TIM8)
 * @param instant_signal: -1.0 ile +1.0 arasinda normalize edilmis anlik sinyal degeri.
 * (Orn: sin(angle) * modulation_index)
 * @param arr_val: Timer'in Auto-Reload Register (Period) degeri (Orn: 1799)
 * * @return unsigned int: CH1 bacagina yuklenen CCR degeri (Debug veya izleme amaclidir)
 */
unsigned int USPWM(TIM_TypeDef *tim, float instant_signal, uint32_t arr_val, float modulation_index)
{
    /* 1. Guvenlik Sinirlamasi (Saturation) */
    /* Sinyal -1.0 ile 1.0 disina cikarsa PWM bozulmamasi icin kirpilir */
    if (instant_signal > 1.0f) instant_signal = 1.0f;
    if (instant_signal < -1.0f) instant_signal = -1.0f;

    /* 2. Duty Cycle Hesaplamasi */
    /* Center Aligned Mod icin formül: Duty = (ARR / 2) * (1 + Sinyal)
       Sinyal 0 iken Duty %50 olur (Tasima dalgasi ile kesisim noktasi) */
    float duty_float = ((float)arr_val / 2.0f) * (1.0f + instant_signal* modulation_index);

    /* 3. Integer Donusumu */
    uint32_t ccr1_val = (uint32_t)duty_float;

    /* 4. Unipolar Mantigi (Leg B Hesabi) */
    /* Leg B (CH2), Leg A (CH1) ile zıt çalışmalıdır.
       Yani sinyal pozitifken CH1 artar, CH2 azalır.
       Matematiksel olarak: CCR2 = ARR - CCR1 */
    uint32_t ccr2_val = arr_val - ccr1_val;

    /* 5. Registerlara Yazma */
    /* Doğrudan Timer Registerlarina erisim (En hizli yontem) */
    tim->CCR1 = ccr1_val;
    tim->CCR2 = ccr2_val;

    /* Opsiyonel: Hesaplanan degeri geri don */
    return ccr1_val;
}
