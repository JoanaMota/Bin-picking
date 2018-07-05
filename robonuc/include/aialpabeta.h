/*
 *  aialpabeta.h
 *  hello
 *
 *  Created by Aron Allen on 11/11/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */


int bestMJab(PGAME orgGame, int secondsToSearch, int plyNodeLimit);

GAME heavyGameFromLightGame(LIGHTGAME lg);
LIGHTGAME lightGameFromHeavyGame(GAME hg);

int stackCompare(const void *a, const void *b);
int stackCompareInverted(const void *a, const void *b);

int scoreCompare(const void *a, const void *b);
int scoreCompareInverted(const void *a, const void *b);
int goDeeper(PGAME hg, int ply);
double scoreGame(PGAME hg, PGAME org, float piecesJumpedInTurn	);

