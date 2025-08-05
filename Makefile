exe := ps1

${exe}: main.cc
	g++ main.cc -std=c++23 -g -Wall -o ${exe}

.PHONY: clean
clean:
	rm -rf ${exe} ${exe}.dSYM
