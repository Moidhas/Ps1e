#include "cpu.hpp"
using namespace std;

int main() {
    try {
        Mips::Cpu cpu;
        cpu.runCpuLoop();
    } catch (const exception &ex) {
        println("{}", ex.what());
    } catch (...) {
        println("uncaught");
    }
}
