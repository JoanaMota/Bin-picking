/*
 *  mover.h
 *  hello
 *
 *  Created by Aron Allen on 30/7/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

BITBOARD moveNorthWest(PGAME game, BITBOARD piece);
BITBOARD moveNorthEast(PGAME game, BITBOARD piece);
BITBOARD moveSouthWest(PGAME game, BITBOARD piece);
BITBOARD moveSouthEast(PGAME game, BITBOARD piece);

void findMoversForGame(PGAME game);