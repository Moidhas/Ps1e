exe := ps1
testExe := test

${exe}: main.cc
	g++ main.cc -std=c++23 -g -Wall -o ${exe}

${testExe}: test.cc Ring.hpp BitUtils.hpp
	g++ test.cc -std=c++23 -g -Wall -o ${testExe}

.PHONY: clean
clean:
	rm -rf ${exe} ${exe}.dSYM ${testExe} ${testExe}.dSYM
