import torch

def get_cuda_memory_usage():
    allocated = torch.cuda.memory_allocated()
    cached = torch.cuda.memory_reserved()
    return allocated, cached


def show_cuda_memory(str=""):
    # print(f"\n{str}")
    # allocated, cached = get_cuda_memory_usage()
    # allocated /= 1024*1024
    # cached /= 1024*1024
    # print("  >> Allocated memory:", allocated, "MB")
    # print("  >> Cached memory:", cached, "MB")
    pass