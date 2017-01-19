import csv
import argparse
import sys

eps = 1e-9

def toFloat(str):
    try:
        return float(str)
    except ValueError:
        return str


parser = argparse.ArgumentParser(
    description='Compare and Output some chart.'
)
parser.add_argument('result', help='Input the file position of compare tool output file (last record only).')
parser.add_argument('threshold', help='Threshold of Error')

args = parser.parse_args()

error_threshold = toFloat(args.threshold);

try:
    with open(args.result, 'rb') as resultFile:

        resultData = csv.reader(resultFile, delimiter=',', quotechar='\n')

        acc_error = 0;
        nums_of_items = 0;

        for num in next(resultData):
            acc_error += toFloat(num)
            nums_of_items += 1

        acc_error /= nums_of_items

        if acc_error <= error_threshold:
            print("Accumaltive Error: {} <= {}, Test Passed".format(acc_error, error_threshold))
            sys.exit(0);
        else:
            print("Accumaltive Error: {} > {}, Test Failed".format(acc_error, error_threshold))
            sys.exit(255);


except IOError as e:
    print(e)
