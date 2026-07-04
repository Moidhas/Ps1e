import argparse
from pathlib import Path

def main():
    # 1. Set up the argument parser
    parser = argparse.ArgumentParser(description="diff checker")
    
    # 2. Add the file path argument
    parser.add_argument("psx", type=str, help="psx file path")
    parser.add_argument("pcsx", type=str, help="pcsx file path")
    
    # 3. Parse the arguments
    args = parser.parse_args()
    
    # 4. Handle the path safely using pathlib
    psx_path = Path(args.psx)
    pcsx_path = Path(args.pcsx)
    
    # 5. Check if the file exists before processing
    if psx_path.is_file() and pcsx_path.is_file():
        with open(psx_path, "r", encoding="utf-8") as psx, open(pcsx_path, "r", encoding="utf-8") as pcsx:
            for psx_line, pcsx_line in zip(psx, pcsx):
                if psx_line != pcsx_line:
                    print(f"Got: {psx_line}\n Wanted: {pcsx_line}")
                    return
            print("NO DIFF!\n")
    else:
        if not psx_path.is_file():
            print(f"Error: The file '{psx_path}' does not exist.")
        else:
            print(f"Error: The file '{pcsx_path}' does not exist.")
            

if __name__ == "__main__":
    main()
