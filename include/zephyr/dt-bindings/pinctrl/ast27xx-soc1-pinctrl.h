/*
 * Copyright (c) ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_AST27XX_SOC1_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_AST27XX_SOC1_H_

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

/* The phyisical ball IDs */
#define C16     0
#define C14     1
#define C11     2
#define D9      3
#define F14     4
#define D10     5
#define C12     6
#define C13     7
#define AC26    8
#define AA25    9
#define AB23    10
#define U22     11
#define V21     12
#define N26     13
#define P25     14
#define N25     15
#define V23     16
#define W22     17
#define AB26    18
#define AD26    19
#define P26     20
#define AE26    21
#define AF26    22
#define AF25    23
#define AE25    24
#define AD25    25
#define AF23    26
#define AF20    27
#define AF21    28
#define AE21    29
#define AE23    30
#define AD22    31
#define AF17    32
#define AA16    33
#define Y16     34
#define V17     35
#define J13     36
#define AB16    37
#define AC16    38
#define AF16    39
#define AA15    40
#define AB15    41
#define AC15    42
#define AD15    43
#define Y15     44
#define AA14    45
#define W16     46
#define V16     47
#define AB18    48
#define AC18    49
#define K13     50
#define AA17    51
#define AB17    52
#define AD16    53
#define AC17    54
#define AD17    55
#define AE16    56
#define AE17    57
#define AB24    58
#define W26     59
#define HOLE0   60
#define HOLE1   61
#define HOLE2   62
#define HOLE3   63
#define W25     64
#define Y23     65
#define Y24     66
#define W21     67
#define AA23    68
#define AC22    69
#define AB22    70
#define Y21     71
#define AE20    72
#define AF19    73
#define Y22     74
#define AA20    75
#define AA22    76
#define AB20    77
#define AF18    78
#define AE19    79
#define AD20    80
#define AC20    81
#define AA21    82
#define AB21    83
#define AC19    84
#define AE18    85
#define AD19    86
#define AD18    87
#define U25     88
#define U26     89
#define Y26     90
#define AA24    91
#define R25     92
#define AA26    93
#define R26     94
#define Y25     95
#define B16     96
#define D14     97
#define B15     98
#define B14     99
#define C17     100
#define B13     101
#define E14     102
#define C15     103
#define D24     104
#define B23     105
#define B22     106
#define C23     107
#define B18     108
#define B21     109
#define M15     110
#define B19     111
#define B26     112
#define A25     113
#define A24     114
#define B24     115
#define E26     116
#define A21     117
#define A19     118
#define A18     119
#define D26     120
#define C26     121
#define A23     122
#define A22     123
#define B25     124
#define F26     125
#define A26     126
#define A14     127
#define E10     128
#define E13     129
#define D12     130
#define F10     131
#define E11     132
#define F11     133
#define F13     134
#define N15     135
#define C20     136
#define C19     137
#define A8      138
#define R14     139
#define A7      140
#define P14     141
#define D20     142
#define A6      143
#define B6      144
#define N14     145
#define B7      146
#define B8      147
#define B9      148
#define M14     149
#define J11     150
#define E7      151
#define D19     152
#define B11     153
#define D15     154
#define B12     155
#define B10     156
#define P13     157
#define C18     158
#define C6      159
#define C7      160
#define D7      161
#define N13     162
#define C8      163
#define C9      164
#define C10     165
#define M16     166
#define A15     167
#define G11     168
#define H7      169
#define H8      170
#define H9      171
#define H10     172
#define H11     173
#define J9      174
#define J10     175
#define E9      176
#define F9      177
#define F8      178
#define M13     179
#define F7      180
#define D8      181
#define E8      182
#define L12     183
#define F12     184
#define E12     185
#define J12     186
#define G7      187
#define G8      188
#define G9      189
#define G10     190
#define K12     191
#define W17     192
#define V18     193
#define W18     194
#define Y17     195
#define AA18    196
#define AA13    197
#define Y18     198
#define AA12    199
#define W20     200
#define V20     201
#define Y11     202
#define V14     203
#define V19     204
#define W14     205
#define Y20     206
#define AB19    207
#define U21     208
#define T24     209
#define V24     210
#define V22     211
#define T23     212
#define AC25    213
#define AB25    214
#define AC24    215

/*
 * Virtual ball identifiers for controller-wide mode/function bits that are
 * not tied to a single physical ball (e.g. USB Port C/D mode select, SGMII
 * enable, PCIe root-complex PERST). Driver uses pin->ball as the ownership
 * key for conflict checking, so each identifier must stay unique.
 */
#define PORTC_MODE      216
#define PORTD_MODE      217
#define SGMII0          218
#define PCIERC2_PERST   219

/* Total number of ball identifiers (physical + virtual), keep in sync above */
#define AST27XX_SOC1_BALL_NUM   220
#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_AST27XX_SOC1_H_ */
