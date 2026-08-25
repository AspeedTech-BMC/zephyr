/*
 * Copyright (c) ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_AST10X0_G2_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_AST10X0_G2_H_

/*
 * bit[31:28] function index
 * bit[27:20] bit offset
 * bit[19:0] SCU register offset
 */
#define SIG_DESC(reg, bit, func) ((reg) | ((bit) << 20) | ((func) << 28))

/*
 * PINCFG_DESC locates a single pin-configuration attribute (e.g. drive
 * strength, bias/pull disable) in the SCU register map, using the same
 * reg/bit-offset layout as SIG_DESC:
 * bit[31:28] field width in bits
 * bit[27:20] bit offset
 * bit[19:0] SCU register offset
 */
#define PINCFG_DESC(reg, bit, width) ((reg) | ((bit) << 20) | ((width) << 28))

/* The physical ball IDs */
#define F14     0
#define E15     1
#define H13     2
#define D17     3
#define D16     4
#define D15     5
#define C15     6
#define F13     7
#define K13     8
#define C14     9
#define A14     10
#define D14     11
#define B14     12
#define B13     13
#define D13     14
#define E13     15
#define R10     16
#define P11     17
#define R11     18
#define U12     19
#define T12     20
#define R12     21
#define N12     22
#define P12     23
#define U13     24
#define T13     25
#define R13     26
#define U14     27
#define T14     28
#define R14     29
#define U15     30
#define P13     31
#define P14     32
#define R17     33
#define R16     34
#define N14     35
#define P16     36
#define N13     37
#define P17     38
#define M14     39
#define N16     40
#define N17     41
#define M15     42
#define M16     43
#define F3      44
#define G5      45
#define G4      46
#define H4      47
#define H5      48
#define J3      49
#define K3      50
#define F4      51
#define G1      52
#define G3      53
#define G2      54
#define H1      55
#define H3      56
#define H2      57
#define R4      58
#define T4      59
#define M17     60
#define L14     61
#define L15     62
#define L16     63
#define L17     64
#define K16     65
#define K14     66
#define K15     67
#define C8      68
#define D8      69
#define A5      70
#define A4      71
#define A3      72
#define A2      73
#define B6      74
#define L3      75
#define N6      76
#define M4      77
#define P1      78
#define N3      79
#define N5      80
#define R1      81
#define L2      82
#define M2      83
#define L4      84
#define N2      85
#define P2      86
#define N4      87
#define T1      88
#define U5      89
#define T5      90
#define R5      91
#define H17     92
#define H16     93
#define H15     94
#define J13     95
#define G17     96
#define H14     97
#define G16     98
#define F17     99
#define G15     100
#define G13     101
#define F16     102
#define E17     103
#define G14     104
#define F15     105
#define E16     106
#define E14     107
#define A9      108
#define B9      109
#define D9      110
#define C9      111
#define A8      112
#define B8      113
#define A7      114
#define E8      115
#define A6      116
#define B7      117
#define B2      118
#define B3      119
#define C6      120
#define D7      121
#define E7      122
#define C7      123
#define B4      124
#define B5      125
#define P5      126
#define P4      127
#define R2      128
#define T2      129
#define U2      130
#define P3      131
#define T3      132
#define U3      133
#define K17     134
#define J17     135
#define J16     136
#define J15     137
#define K12     138
#define C5      139
#define D4      140
#define D6      141
#define C4      142
#define B1      143
#define T16     144
#define N15     145
#define K4      146
#define K5      147
#define L5      148
#define M5      149
#define M3      150
#define J1      151
#define J2      152
#define K1      153
#define L1      154
#define K2      155
#define N1      156
#define M1      157
#define C2      158
#define C1      159
#define D3      160
#define D1      161
#define E1      162
#define E3      163
#define F1      164
#define U4      165
#define F5      166
#define D2      167
#define E2      168
#define E4      169
#define F2      170
#define F12     171
#define A13     172
#define E12     173
#define B12     174
#define G12     175
#define C12     176
#define D12     177
#define C13     178
#define A12     179
#define D11     180
#define B11     181
#define C11     182
#define A11     183
#define E10     184
#define A10     185
#define B10     186
#define R9      187
#define P10     188
#define J5      189
#define J4      190
#define E5      191
#define F6      192
#define J14     193
#define PORTA_MODE      194
#define PORTB_MODE      195

/* Total number of ball identifiers (physical + virtual), keep in sync above */
#define AST10X0_G2_BALL_NUM     196

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_AST10X0_G2_H_ */
