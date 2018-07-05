//
//  alphabeta.h
//  hello
//
//  Created by Aron Allen on 18/4/11.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#include "main.h"


int bestMJ(PGAME orgGame, PGAME game, int depth, int f);
int bestMJalt(PGAME orgGame, PGAME game, int d, int f, int returnScore);
GAME heavyGameFromLightGame(LIGHTGAME lg);
LIGHTGAME lightGameFromHeavyGame(GAME hg);
int scoreGames(PGAME o, PGAME n);