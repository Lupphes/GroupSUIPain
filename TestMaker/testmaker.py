with open("test.sh", "w") as testFile:
    for i in range(1,101):
        print(f"echo \"--------------------------------------------TEST {i}--------------------------------------------\"",file=testFile)
        print(f"./fc-sui 1 {i} --easy-mode 10 --solver bfs",file=testFile)
        print(f"echo \"------------------------------------------------------------------------------------------------\"",file=testFile)