def printSpeed(msg):
    # ANSI escape codes for colors and style
    RED = "\033[91m"
    GREEN = "\033[92m"
    CYAN = "\033[96m"
    YELLOW = "\033[93m"
    RESET = "\033[0m"
    BOLD = "\033[1m"

    print(f"""
{BOLD}{CYAN}         WHEEL SPEEDS{RESET}
    {YELLOW}┌───────────────────┐{RESET}
    {YELLOW}│{RESET}  [1]         [2]  {YELLOW}│{RESET}
    {YELLOW}│{RESET} ({GREEN}{msg[0]:03}{RESET})       ({GREEN}{msg[1]:03}{RESET}) {YELLOW}│{RESET}
    {YELLOW}│{RESET}    \\       /      {YELLOW}│{RESET}
    {YELLOW}│{RESET}     \\     /       {YELLOW}│{RESET}
    {YELLOW}│{RESET}     /     \\       {YELLOW}│         {RESET}
    {YELLOW}│{RESET}    /       \\      {YELLOW}│{RESET}
    {YELLOW}│{RESET} ({GREEN}{msg[3]:03}{RESET})       ({GREEN}{msg[2]:03}{RESET}) {YELLOW}│{RESET}
    {YELLOW}│{RESET}  [4]         [3]  {YELLOW}│{RESET}
    {YELLOW}└───────────────────┘{RESET}
""")


printSpeed([0, 1, 10, 100])
