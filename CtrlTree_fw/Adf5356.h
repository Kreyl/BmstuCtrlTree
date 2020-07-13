/*
 * Adf5356.h
 *
 *  Created on: 6 июн. 2020 г.
 *      Author: layst
 */

#pragma once

#include <math.h>

/*
Проверка-------------------------------------------------------------------
При вызоыве RegistersCalc(20d,500, 0, 5939.58d, out r0, out r1, out r2, out r10, out rD) должны вернуться следующие Регистры:
R0: 0x302510, r1: 0xF53F7C1, r2: 0x1D007D2, rA: 0xC0067A, rD: 0x00000D

При вызоыве RegistersCalc(20d,500,130500, 6287.40d, out r0, out r1, out r2, out r10, out rD) должны вернуться следующие Регистры:
R0: 0x302740, r1: 0xC0C7E21, r2: 0x4F82712, rA: 0xC0067A, rD: 0x00000D

При вызоыве RegistersCalc(20d,500,0, 6287.40d, out r0, out r1, out r2, out r10, out rD) должны вернуться следующие Регистры:
R0: 0x302740, r1: 0xBD70A31, r2: 0x540192, rA: 0xC0067A, rD: 0x00000D
*/

class Adf5356_t {
private:
    PinOutput_t LE{GPIOA, 4, omPushPull}; // Latch Enable
    Spi_t ISpi{SPI1};
    // Regs calculation
    const double Fref = 100.0; // частота опоры, в метрологии 100 МГц
    const double Step = 500.0; // шаг перестройки, в метрологии 500 Гц
    const double Fdoppler = 0; // частота Допплера, в метрологии - 0 Гц
    uint32_t r[14];

    // fpfd Частота фазового детектора, МГЦ
    int Reg10Calc(double fpfd) {
        //Reserved
        const int db31_db14 = 0xC00000;//0b00000000110000000000000000000000;
        //ADC Conversion Enable
        const int db5 = 0x20;//0b00000000000000000000000000100000;
        //ADC Enable
        const int db4 = 0x10;//0b00000000000000000000000000010000;
        //Control Bits
        const int db3_db0 = 0xA;// 0b00000000000000000000000000001010;
        //ADC Clock Divider (ADC_CLK_DIV)
        int db13_db6 = (int)ceil(((fpfd * 1e3 / 100) - 2) / 4);
        if (db13_db6 > 255) db13_db6 = 255;
        return db31_db14 | db13_db6 << 6 | db5 | db4 | db3_db0;
    }

    // Поиск наибольшего общего делителя </summary>
    int GetNOD(int a, int b)  {
        while(b != 0) {
            int temp = b;
            b = a % b;
            a = temp;
        }
        return a;
    }
public:
    bool Initialized = false; // Send R0 twice after power-on
    double fref, step, fd, fvco;
    void Init() {
        // Init GPIOs
        LE.Init();
        LE.SetHi();
        chThdSleepMilliseconds(7);
        // Init SPI
        PinSetupAlterFunc(GPIOA, 5, omPushPull, pudNone, AF5, psVeryHigh); // SCK
        PinSetupAlterFunc(GPIOA, 7, omPushPull, pudNone, AF5, psVeryHigh); // MOSI
        rccEnableSPI1(FALSE);
        ISpi.Setup(boMSB, cpolIdleLow, cphaFirstEdge, 8000000, bitn16);
        ISpi.Enable();
    }

    void WriteReg(uint32_t RegValue) { // 28xValue, 4xAddr
        // Delay before Reg0
        if((RegValue & 0b1111UL) == 0) chThdSleepMilliseconds(1);
        LE.SetLo();
        ISpi.ReadWriteWord((RegValue >> 16) & 0xFFFF);
        ISpi.ReadWriteWord(RegValue & 0xFFFF);
        LE.SetHi();
    }

    /* Расчет массива значений регистров
    <param name="fref">Частота опоры, МГц (в бурчуне 20, в метрологии 100)</param>
    <param name="step">Шаг перестройки, Гц (по умолчанию 500)</param>
    <param name="fd">Частота Допплера, Гц (в бурчуне изменяется от -150000 до 150000 с шагом 500, в метрологии 0)</param>
    <param name="fvco">Частота гетеродина, МГц (для ГЛОНАСС L1 - 5939.58, L2 - 6287.40, L3 - 6338.55, для KU - 6342.5) */
    void CalcRegs() {
        r[3] = 0x3;
        r[4] = 0x32008B84;
        r[5] = 0x800025;
        r[6] = 0x35028076;
        r[7] = 0x60000E7;
        r[8] = 0x15596568;
        r[9] = 0x20153CC9;
        r[11] = 0x61200B;
        r[12] = 0x15FC;

        // Частота фазового детектора, МГц
        double fpfd = fref / 2;
        // Отношение частоты гетеродина к частоте фазового детектора
        double n = (fvco + (fd / 1e6)) / fpfd;
        // целая часть - пишется в регистр 0.
        int iNT = (int)n;
        // номер регистра
        int registerNum = 0x000000;
        // значение регистра 0
        r[0] = (iNT << 4) | 0x300000 | registerNum;
        // дробная часть n
        double remain1 = n - iNT;
        // дробная часть смещенная
        double frac = remain1 * 16777216.0;
        // дробная часть увеличенная, переведенная в целое
        int frac1 = (int)frac;
        registerNum = 0x000001;
        //значение регистра 1
        r[1] = frac1 << 4 | registerNum;

        // дробная часть, оставшаяся от frac после отделения frac1
        // double remain2 = Math.Round(frac - frac1, 5);
        double remain2 = round((frac - frac1) * 100000.0) / 100000.0;
        int mod2 = (int)(pow(fpfd, 7.0) / step);
        int frac2 = (int)(remain2 * mod2);

        // Поиск наибольшего общего делителя
        int nod = GetNOD(frac2, mod2);
        // Сокращаем
        frac2 /= nod;
        mod2 /= nod;

        //стираем ненужниые биты
        int lSB_Frac = frac2 & 0b00000000000000000011111111111111;
        int lSB_Mod  =  mod2 & 0b00000000000000000011111111111111;

        registerNum = 0x000002;
        //значение регистра 2
        r[2] = ((lSB_Frac << 18) | lSB_Mod << 4) | registerNum;

        // стираем ненужниые биты
        int mSB_Fraq = frac2 & 0b00001111111111111100000000000000;
        int mSB_Mod  =  mod2 & 0b00001111111111111100000000000000;

        registerNum = 0x00000D;
        //значение регистра D
        r[13] = ((mSB_Fraq << 4) | (mSB_Mod >> 10)) | registerNum;

        r[10] = Reg10Calc(fpfd);
    }

    void SetRegs() {
        if(Initialized) {
            WriteReg(r[13]);
            WriteReg(r[10]);
            WriteReg(r[ 2]);
            WriteReg(r[ 1]);
            chThdSleepMilliseconds(1);
            WriteReg(r[ 0]);
        }
        // Init it
        else {
            WriteReg(r[13]);
            WriteReg(r[12]);
            WriteReg(r[11]);
            WriteReg(r[10]);
            WriteReg(r[ 9]);
            WriteReg(r[ 8]);
            WriteReg(r[ 7]);
            WriteReg(r[ 6]);
            WriteReg(r[ 5]);
            WriteReg(r[ 4]);
            WriteReg(r[ 3]);
            WriteReg(r[ 2]);
            WriteReg(r[ 1]);
            chThdSleepMilliseconds(1); // Ensure that >16 ADC clock cycles elapse between the write of Register 10 and Register 0.
            WriteReg(r[ 0]);
            Initialized = true;
        }
    }

    void PrintRegs() {
        for(int i=0; i<14; i++) Printf("r%u:0x%X; ", i, r[i]);
        PrintfEOL();
    }
};
