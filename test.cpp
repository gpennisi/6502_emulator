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



    std::vector<uint8_t> program4 = {
        // --- SETUP ---
        0xA9, 0x55,          // [IMM]      LDA #$55   (Load 0x55 into A)
        0x85, 0x10,          // [ZP]       STA $10    (Store 0x55 in Zero Page $10)
        0x8D, 0x00, 0x01,    // [ABS]      STA $0100  (Store 0x55 in Absolute $0100)
        
        0xA9, 0x00,          // [IMM]      LDA #$00
        0x85, 0x40,          // [ZP]       STA $40    (Store low byte of pointer at $40)
        0xA9, 0x01,          // [IMM]      LDA #$01
        0x85, 0x41,          // [ZP]       STA $41    (Store high byte of pointer at $41)
                             //            (Pointer at $0040 now points to $0100)

        // --- TEST 1: Register/Implied ---
        0xAA,                // [IMP]      TAX        (Transfer A [0x01] to X)
        0xE8,                // [IMP]      INX        (Increment X, X is now 0x02)
        0x8A,                // [IMP]      TXA        (Transfer X to A, A is now 0x02)

        // --- TEST 2: Accumulator ---
        0xA9, 0x55,          // [IMM]      LDA #$55   (Reset A)
        0x4A,                // [ACC]      LSR A      (Logical Shift Right A, 0x55 -> 0x2A)
        0x0A,                // [ACC]      ASL A      (Arithmetic Shift Left A, 0x2A -> 0x54)

        // --- TEST 3: Zero Page Offsets ---
        0xA2, 0x01,          // [IMM]      LDX #$01
        0xB5, 0x0F,          // [ZPX]      LDA $0F, X (Read from $0F + 1 = $10. A should be 0x55)
        
        0xA0, 0x01,          // [IMM]      LDY #$01
        0xB6, 0x0F,          // [ZPY]      LDX $0F, Y (Read from $0F + 1 = $10. X should be 0x55)
                             // Note: ZP,Y is rare. Only LDX and STX use it.

        // --- TEST 4: Absolute Offsets ---
        0xA2, 0x01,          // [IMM]      LDX #$01
        0xBD, 0xFF, 0x00,    // [ABSX]     LDA $00FF, X (Read $00FF + 1 = $0100. A should be 0x55)
        
        0xA0, 0x01,          // [IMM]      LDY #$01
        0xB9, 0xFF, 0x00,    // [ABSY]     LDA $00FF, Y (Read $00FF + 1 = $0100. A should be 0x55)

        // --- TEST 5: Indirect Addressing (The hard ones) ---
        
        // Indexed Indirect (Indirect, X)
        // We use pointer at $40. We want to read ($40 + X). 
        // Let X=0. We read the pointer at $40, which points to $0100.
        0xA2, 0x00,          // [IMM]      LDX #$00
        0xA1, 0x40,          // [INDX]     LDA ($40, X) (Dereference pointer at $40. Load $0100. A=0x55)

        // Indirect Indexed (Indirect), Y
        // We read pointer at $40 ($0100), then add Y to the address.
        // Let Y=0. Target is $0100 + 0 = $0100.
        0xA0, 0x00,          // [IMM]      LDY #$00
        0xB1, 0x40,          // [INDY]     LDA ($40), Y (Dereference ptr at $40, add Y. Load $0100. A=0x55)

        // --- TEST 6: Relative Branching ---
        0xA9, 0x01,          // [IMM]      LDA #$01
        0xC9, 0x01,          // [IMM]      CMP #$01     (Compare A with 1. Sets Zero Flag)
        0xF0, 0x01,          // [REL]      BEQ +1       (Branch Equal to next byte. Skips the BRK)
        0x00,                // [IMP]      BRK          (Should be skipped)
        
        // --- TEST 7: Absolute Indirect (JMP only) ---
        // Setup a pointer at $0050 that points to the NOP below ($0648 approximately)
        // Since calculating exact JMP addresses in a vector is tricky, 
        // we usually hardcode the destination or use a label in an assembler.
        // For this test, we will just JMP Absolute to the end.
        0x4C, 0x48, 0x06,    // [ABS]      JMP $0648    (Jump to the NOP below)

        0x00,                // [IMP]      BRK          (Trap if JMP fails)

        // $0648
        0xEA,                // [IMP]      NOP          (Land here)
        0x00                 // [IMP]      BRK          (End)
    };



    bus.loadProgram(program4, 0x0600);
    uint16_t startAddress = 0x0600;
    cpu.disassemble(startAddress, startAddress+program4.size() );

    return 0;
}