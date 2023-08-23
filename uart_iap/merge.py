#!/usr/bin/python
import sys


def usage():
    print("Usage:")
    print(f" python {sys.argv[0]} [-a|--inA <FILE>] [-b|--sizeA <SIZE>] [-c|--inB <FILE>] [-d|--sizeB <SIZE>] [-o|--out <FILE>]")


if __name__ == "__main__":
    import getopt

    try:
        opts, args = getopt.getopt(sys.argv[1:], "a:b:c:d:o:", ["inA=", "inB=", "sizeA=", "sizeB=", "out="])
    except getopt.GetoptError as err:
        print(err)
        usage()
        exit(2)

    bin1 = "../bootloader/emStudio/Output/Release/Exe/bootloader.bin"
    bin2 = "../factory/emStudio/Output/Release/Exe/factory.bin"
    out = 'output.bin'
    bin1_size = 0x4000
    bin2_size = 0x4000

    for op, val in opts:
        if op in ("-a", "--inA"):
            bin1 = val
            print("get -a: %s" % bin1)
        elif op in ("-b", "--sizeA"):
            bin1_size = int(val, 16)
            print("get -b: %d" % bin1_size)
        elif op in ("-c", "--inB"):
            bin2 = val
            print("get -c: %s" % bin2)
        elif op in ("-d", "--sizeB"):
            bin2_size = int(val, 16)
            print("get -d: %d" % bin2_size)
        elif op in ("-o", "--out"):
            out = val
            print("get -o: %s" % out)
        else:
            assert False, "UNHANDLED OPTION"

    with open(bin1, "rb") as f_obj:
        bootloader = f_obj.read()
        bootloader += b'\xff' * (bin1_size - len(bootloader))

    with open(bin2, "rb") as f_obj:
        factory = f_obj.read()
        factory += b'\xff' * (bin2_size - len(factory))

    with open(out, 'wb') as f_obj:
        bin_data = bootloader + factory
        f_obj.write(bin_data)
        