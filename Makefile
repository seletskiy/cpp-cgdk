OBJS=Runner.o RemoteProcessClient.o model/PlayerContext.o model/Game.o model/Unit.o model/Move.o model/World.o model/Trooper.o model/Player.o model/Bonus.o MyStrategy.o Strategy.o csimplesocket/SimpleSocket.o csimplesocket/HTTPActiveSocket.o csimplesocket/ActiveSocket.o csimplesocket/PassiveSocket.o
CC=g++
DEBUG=-g
CFLAGS=-Wall -c $(DEBUG) -lm -x c++ -D_LINUX -s
LFLAGS=-Wall $(DEBUG)

MyStrategy: $(OBJS)
	$(CC) $(LFLAGS) $(OBJS) -o MyStrategy

%.o: %.cpp
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	find . -name '*.o' -delete