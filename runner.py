import subprocess
import time

binary_path = "./Colortracking"  # replace with the path to your binary

while True:
    result = subprocess.run(binary_path)
    
    if result.returncode == -6:
        print("Exit code 134 detected. Waiting 2 seconds and restarting...")
        time.sleep(2)
    else:
        print(f"Exited with code {result.returncode}. Stopping.")
        break

