import subprocess

def is_script_running(script_name):
    # Run the ps command to check if the script is running
    result = subprocess.run(['ps', 'aux'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    # Check if the script is found in the list of processes
    return script_name in result.stdout.decode()

script_name = r'2025_7476_reefscape\limelight\python\algae\orange_algae_classification.py'
print(is_script_running(script_name))
