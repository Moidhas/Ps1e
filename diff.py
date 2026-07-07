import argparse
from pathlib import Path

def main():
    parser = argparse.ArgumentParser(description="diff checker")
    
    parser.add_argument("psx", type=str, help="psx file path")
    parser.add_argument("pcsx", type=str, help="pcsx file path")
    
    args = parser.parse_args()
    
    psx_path = Path(args.psx)
    pcsx_path = Path(args.pcsx)
    
    if psx_path.is_file() and pcsx_path.is_file():
        with open(psx_path, "r", encoding="utf-8") as psx, open(pcsx_path, "r", encoding="utf-8") as pcsx:
            for line_num, (psx_line, pcsx_line) in enumerate(zip(psx, pcsx), start=1):
                    if psx_line != pcsx_line:
                        got_clean = psx_line.rstrip('\r\n')
                        wanted_clean = pcsx_line.rstrip('\r\n')
                        print(f"Line {line_num:<6} | Got:    {got_clean}")
                        print(f"Line {line_num:<6} | Wanted: {wanted_clean}")
                        print("-" * 40)
    else:
        if not psx_path.is_file():
            print(f"Error: The file '{psx_path}' does not exist.")
        else:
            print(f"Error: The file '{pcsx_path}' does not exist.")
            

if __name__ == "__main__":
    main()
