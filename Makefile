EXE := ./build/ps1
TEST_EXE := ./build/test
SRC := $(wildcard ./src/*.cc)
TEST := ./tests/test.cc

${EXE}: ${SRC}
	g++ ${SRC} -std=c++23 -g -Wall -o ${EXE}

${TEST_EXE}: ${SRC} ${TEST}
	g++ ${SRC} ${TEST} -std=c++23 -g -Wall -o ${TEST_EXE}

.PHONY: clean
clean:
	rm -rf ${EXE} ${EXE}.dSYM ${TEST_EXE} ${TEST_EXE}.dSYM
