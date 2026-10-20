/******************************************************************************
 * @file            opcodes.c
 *****************************************************************************/
#include "opcodes.h"

const char *opcode_name(uint8_t op) {
    switch (op) {
        case 0x00: return "ADD r/m8,r8";
        case 0x01: return "ADD r/m16,r16";
        case 0x02: return "ADD r8,r/m8";
        case 0x03: return "ADD r16,r/m16";
        case 0x04: return "ADD AL,imm8";
        case 0x05: return "ADD AX,imm16";
        case 0x28: return "SUB r/m8,r8";
        case 0x29: return "SUB r/m16,r16";
        case 0x2A: return "SUB r8,r/m8";
        case 0x2B: return "SUB r16,r/m16";
        case 0x2C: return "SUB AL,imm8";
        case 0x2D: return "SUB AX,imm16";
        case 0x30: return "XOR r/m8,r8";
        case 0x31: return "XOR r/m16,r16";
        case 0x32: return "XOR r8,r/m8";
        case 0x33: return "XOR r16,r/m16";
        case 0x34: return "XOR AL,imm8";
        case 0x35: return "XOR AX,imm16";
        case 0x38: return "CMP r/m8,r8";
        case 0x39: return "CMP r/m16,r16";
        case 0x3A: return "CMP r8,r/m8";
        case 0x3B: return "CMP r16,r/m16";
        case 0x3C: return "CMP AL,imm8";
        case 0x3D: return "CMP AX,imm16";
        case 0x7C: return "JL/JNGE rel8";
        case 0x82: return "Grp1 r/m8,imm8";
        case 0x86: return "XCHG r/m8,r8";
        case 0x87: return "XCHG r/m16,r16";
        case 0x88: return "MOV r/m8,r8";
        case 0x89: return "MOV r/m16,r16";
        case 0x8A: return "MOV r8,r/m8";
        case 0x8B: return "MOV r16,r/m16";
        case 0x90: return "NOP";
        case 0xC3: return "RET";
        case 0xC6: return "MOV r/m8,imm8";
        case 0xC7: return "MOV r/m16,imm16";
        case 0xC8: return "ENTER imm16,imm8";
        case 0xCD: return "INT imm8";
        case 0xE8: return "CALL rel16";
        case 0xE9: return "JMP rel16";
        case 0xEB: return "JMP short";
        case 0xF4: return "HLT";
        default: return "UNKNOWN";
    }
}
