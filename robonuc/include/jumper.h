/*
 *  jumper.h
 *  hello
 *
 *  Created by Aron Allen on 31/7/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

int jumpNorthWest(PGAME game, PMJ jump);
int jumpNorthEast(PGAME game, PMJ jump);
int jumpSouthWest(PGAME game, PMJ jump);
int jumpSouthEast(PGAME game, PMJ jump);

void findJumpsForPiece(PGAME game, BITBOARD piece);

void findJumpersForGame(PGAME game);
void newMJ(PMJ jump);