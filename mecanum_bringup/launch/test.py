import os

if __name__ == "__main__":
    proj_root = os.path.dirname(os.path.abspath(os.path.join(__file__, "../..")))
    map_location = os.path.join(proj_root, "mecanum_navigation", "maps")
    map_file_name = os.path.join(map_location, "mapa")
    print(map_file_name)
