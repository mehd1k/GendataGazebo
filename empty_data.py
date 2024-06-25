import os
import glob

def remove_files_in_folders(directory):
    # List all subfolders in the given directory
    subfolders = [f.path for f in os.scandir(directory) if f.is_dir()]
    # subfolders = ['/home/mehdi/data_gazebo/range_plots', '/home/mehdi/data_gazebo/label_npy', '/home/mehdi/data_gazebo/label', '/home/mehdi/data_gazebo/dif_sig']
    for folder in subfolders:
        # Get all files in the current folder
        files = glob.glob(os.path.join(folder, '*'))
        
        # Remove each file
        for file in files:
            try:
                os.remove(file)
                print(f"Removed file: {file}")
            except Exception as e:
                print(f"Error removing file: {file}, {e}")

# Example usage
directory = '/home/mehdi/data_gazebo'
remove_files_in_folders(directory)
