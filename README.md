# 6502_emulator
--- THIS PROJECT IS A WIP ---

This is a work in progress project I am using to learn C++ and improve my understanding of the MOS 6502 processor.

To build this emulator I am using documentation found online

https://www.nesdev.org/obelisk-6502-guide/reference.html

http://www.6502.org/users/obelisk/6502/instructions.html

https://www.masswerk.at/6502/6502_instruction_set.html

Looking at other implementation like https://github.com/gianlucag/mos6502 has been incredibly useful.

I have also been testing my results against https://skilldrick.github.io/easy6502/, which is an amazing resource to understand how assembly language works in 6502.

below some comparisons with easy6502, where results are mostly consistent, except for the logic with BRK which in my current tests exits the program before advancing the PC.

<img width="1164" height="652" alt="Untitled1" src="https://github.com/user-attachments/assets/4e87e556-e520-434d-bbe0-2dab422d5b06" />
<img width="1164" height="652" alt="Untitled2" src="https://github.com/user-attachments/assets/7ea630fb-90bc-4495-ae33-d3d93c86d07b" />
<img width="1164" height="652" alt="Untitled3" src="https://github.com/user-attachments/assets/3de0aee4-83ba-4e79-a779-14c88ce9d6ce" />


