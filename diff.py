from collections.abc import Iterable
import itertools
import argparse
from pathlib import Path

def diff(sliced: Iterable[tuple[str, str]], start: int):
    for line_num, (psx_line, pcsx_line) in enumerate(sliced, start=start):
        if psx_line != pcsx_line:
            got_clean = psx_line.rstrip('\r\n')
            wanted_clean = pcsx_line.rstrip('\r\n')
            print(f"Line {line_num:<6} | Got:    {got_clean}")
            print(f"Line {line_num:<6} | Wanted: {wanted_clean}")
            print("-" * 40)

def diff_pc(sliced: Iterable[tuple[str, str]], start: int):
    for line_num, (psx_line, pcsx_line) in enumerate(sliced, start=start):
        if psx_line.split()[:2] != pcsx_line.split()[:2]:
            got_clean = psx_line.rstrip('\r\n')
            wanted_clean = pcsx_line.rstrip('\r\n')
            print(f"Line {line_num:<6} | Got:    {got_clean}")
            print(f"Line {line_num:<6} | Wanted: {wanted_clean}")
            print("-" * 40)
            return

def main():
    parser = argparse.ArgumentParser(description="diff checker")
    
    _ = parser.add_argument("psx", type=str, help="psx file path")
    _ = parser.add_argument("pcsx", type=str, help="pcsx file path")
    _ = parser.add_argument("--start", type=int, default=1, help="diff starts at line number 'start'")
    _ = parser.add_argument("--end", type=int, default=None, help="diff ends at line number 'end'")
    _ = parser.add_argument('-p', '--pc_diff', action='store_true', help="Only look for differences in pc and opcode.")
    _ = parser.add_argument("--start_pcsx", type=int, default=None, help="pcsx diff starts at line number 'start_pcsx'")
    
    args = parser.parse_args()
    
    psx_path = Path(args.psx) 
    pcsx_path = Path(args.pcsx) 
    start = int(args.start)
    end = args.end
    pc_diff = args.pc_diff

    start_pcsx = int(args.start_pcsx) if args.start_pcsx is not None else start

    if psx_path.is_file() and pcsx_path.is_file():
        with open(psx_path, "r", encoding="utf-8") as psx, open(pcsx_path, "r", encoding="utf-8") as pcsx:
            stop = (end - start + 1) if end is not None else None

            psx_sliced = itertools.islice(psx, start - 1, (start - 1 + stop) if stop is not None else None)
            pcsx_sliced = itertools.islice(pcsx, start_pcsx - 1, (start_pcsx - 1 + stop) if stop is not None else None)

            zipped = zip(psx_sliced, pcsx_sliced)

            if pc_diff:
                diff_pc(zipped, start)
            else:
                diff(zipped, start)

    else:
        if not psx_path.is_file():
            print(f"Error: The file '{psx_path}' does not exist.")
        else:
            print(f"Error: The file '{pcsx_path}' does not exist.")
            

if __name__ == "__main__":
    main()
