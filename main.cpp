#include "Cpu.h"
#include <iostream>

// init hardware (outside so it's on heap)
Bus bus;
Cpu cpu(bus);


void execProgram(const std::vector<uint8_t>& program, bool debug=false, bool step=false)
{
    // load program into mem
    bus.loadProgram(program, 0x0600, debug);

    // reset vector (where PC will start from)
    bus.write(0x00, 0xFFFC);
    bus.write(0x06, 0xFFFD);

    // reset cpu
    cpu.reset();

    // execute program

    while (true) {
        
        cpu.cycle(debug);

        //step by step debug
        if (step) { 
            std::cin.get(); 
            cpu.printCpuState();
        }

        // exit when we hit break (for thsi test)
        if (bus.read(cpu.getPC()) == 0x00) break;

    }
}


int main()
{
    // fillup instructions
    cpu.initInstructions();
    //std::vector<uint8_t> program;
   
    //  PROGRAM 1
    std::vector<uint8_t> program1 = {
        0xA9, 0x05,          // LDA #$05
        0x8D, 0x00, 0x01,    // STA $0100  
        0x8D, 0x10, 0x01,    // STA $0110
        0xAD, 0x00, 0x01,    // LDA $0100
        0x18,                // CLC
        0xE9, 0x01,          // SBC #$01
        0x8D, 0x00, 0x01,    // STA $0100
        0x18,                // CLC
        0x00                 // BRK  
    };

    //  PROGRAM 2
    std::vector<uint8_t> program2 = {
        0xA9, 0x20,          // LDA #$20
        0x8D, 0x11, 0X00,    // STA $0011 
        0x8D, 0x12, 0x00,    // STA $0012

        0xA2, 0x05,          // LDX #$05

        0xAD, 0x12, 0x00,    // LDA $0012
        0x18,                // CLC
        0x69, 0x01,          // ADC #$01
        0x8D, 0x12, 0x00,    // STA $0012
        0xCA,                // DEX

        0xD0, 0xF4,          // BNE ( 12 bytes from LDA $0012, -12 in two's compl is 0xF4)
        0xA0, 0x01,          // LDY #$01

        0x00                 // BRK  
    };
    
    //  PROGRAM 3
    std::vector<uint8_t> program3 = {
        0xA9, 0x02,          // LDA #$02
        0x49, 0xFF,          // EOR #$FF 
        0x18,                // CLC
        0x69, 0x01,          // ADC #$01
        0x8D, 0x00, 0x02,    // STA $0200
 
        0xA9, 0x7F,          // LDA #$7F
        0x49, 0xFF,          // EOR #$FF 
        0x18,                // CLC
        0x69, 0x01,          // ADC #$01

        0x18,                // CLC
        0x6D, 0x00, 0x02,    // ADC $0200

        0xD0, 0x01,          // BNE (skip BRK if successfull)
        0x00,                // BRK

        0xB8,                // CLV
        0x8D, 0x10, 0x02,    // STA $0210

        0x00                 // BRK
    };

    execProgram(program1);
    cpu.printCpuState();
    bus.printValueAtMemory(0x0100);
    bus.printValueAtMemory(0x0110);

    execProgram(program2);
    cpu.printCpuState();
    bus.printValueAtMemory(0x0011);
    bus.printValueAtMemory(0x0012);

    execProgram(program3);
    cpu.printCpuState();
    bus.printValueAtMemory(0x0200);
    bus.printValueAtMemory(0x0210);



    return 0;
}