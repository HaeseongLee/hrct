import yaml
import os
import shutil

def read_robot_state(file_path):
    with open(file_path, 'r') as file:
        yaml_data = yaml.safe_load(file)
        return yaml_data

def extract_robot_info(yaml_data):
    dir_info = {}
    
    dir_info['robot_state'] = yaml_data['robot_state']
    dir_info['camera_state'] = yaml_data['camera_state']
    dir_info['save_directory'] = yaml_data['save_directory'] 
    return dir_info


# 메인 함수
def main():
    task_name = "block_sorting.yaml"
    root =  os.path.dirname(os.path.abspath(__file__))
    yaml_file = os.path.join(root, "../config/", task_name)

    yaml_data = read_robot_state(yaml_file)
    dir_info = extract_robot_info(yaml_data)

    # generate the destination directory
    os.makedirs(dir_info['save_directory'], exist_ok=True)

    # data 폴더 내의 ep_* 폴더 목록 가져오기
    ep_folders = [d for d in os.listdir(dir_info['save_directory']) if d.startswith("ep_") 
                  and os.path.isdir(os.path.join(dir_info['save_directory'], d))]

    file_num = len(ep_folders) + 1

    # 새로운 ep_N 폴더 생성
    new_ep_folder = os.path.join(dir_info['save_directory'], f"ep_{file_num:02d}")
    os.makedirs(new_ep_folder)

    print(new_ep_folder)

    for src in dir_info['robot_state']:
        shutil.move(src, new_ep_folder)

    for src in dir_info['camera_state']:
        shutil.move(src, new_ep_folder)


if __name__ == "__main__":
    main()
