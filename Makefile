EXE := ./build/ps1
TEST_EXE := ./build/test
SRC := $(wildcard ./src/*.cc)
TEST_SRC := $(filter-out %main.cc, ${SRC})
TEST := ./tests/test.cc

.PHONY: clean, test

${EXE}: ${SRC}
	g++ ${SRC} -std=c++23 -g -Wall -o ${EXE}

test: ${TEST_EXE}
${TEST_EXE}: ${TEST_SRC} ${TEST}
	g++ ${TEST_SRC} ${TEST} -std=c++23 -g -Wall -o ${TEST_EXE}

clean:
	rm -rf ${EXE} ${EXE}.dSYM ${TEST_EXE} ${TEST_EXE}.dSYM
