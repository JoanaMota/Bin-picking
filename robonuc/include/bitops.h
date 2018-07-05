/*
 *  bitops.h
 *  hello
 *
 *  Created by Aron Allen on 30/7/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */


//Bit Rotations, single bits only!
BITBOARD northWestRotate(BITBOARD bitboard);
BITBOARD northEastRotate(BITBOARD bitboard);
BITBOARD southEastRotate(BITBOARD bitboard);
BITBOARD southWestRotate(BITBOARD bitboard);


//Bit counting.
int bitsInBitboard(BITBOARD bitboard);

//!!Only for 1-bit bitboads, opposite of bitboardForRealPosition[i].
int realPositionForBitboard(BITBOARD bitboard);



char pieceAtPosition (PGAME game, BITBOARD position);