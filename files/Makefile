CC=gcc
CFLAGS=-L. -lInvnAlgoRangeFinder -lInvnAlgoFloorTypeFxp -lInvnAlgoCliffDetection -lInvnAlgoObstaclePosition -lm
DEPS = tdk-chx01-get-data.c
OBJ = tdk-chx01-get-data
LIB = libtdk-chx01-get-data.so

all: $(OBJ) $(LIB)

%.o: %.c $(DEPS)
		$(CC) -c -fPIC -o $@ $< $(CFLAGS)

$(OBJ): $(OBJ).o
		$(CC) -o $@ $^ $(CFLAGS)

$(LIB): $(OBJ).o
		$(CC) -shared -o $@ $^

.PHONY: clean

clean:
		rm -f *.o *~ core $(INCDIR)/*~ $(OBJ) $(LIB)