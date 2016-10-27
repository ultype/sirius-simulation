import csv
import argparse

def toFloat(str):
    try:
        return float(str)
    except ValueError:
        return str


parser = argparse.ArgumentParser(
    description='Compare and Output some chart.'
)
parser.add_argument('golden', help='Input the file position of golden file.')
parser.add_argument('target', help='Input the file position of target file.')

parser.add_argument('-l', '--last',
    help='Compare the last data between golden and target file.',
    action='store_true'
)

args = parser.parse_args()

try:
    with open(args.golden, 'rb') as goldenFile,            \
         open(args.target, 'rb') as targetFile,            \
         open('result.csv','wb') as outoutFile:
        goldenData = csv.reader(goldenFile, delimiter=',', quotechar='\n')
        targetData = csv.reader(targetFile, delimiter=',', quotechar='\n')

        goldenArray = [];
        targetArray = [];
        outputArray = [];

        if(args.last):
            print('jizz')
        else:
            for row in goldenData:
                goldenArray.append([]);
                for num in row:
                    goldenArray[-1].append(toFloat(num))

            for row in targetData:
                targetArray.append([]);
                for num in row:
                    targetArray[-1].append(toFloat(num))

            for idxRow, goldenRow in enumerate(goldenData):
                outputArray.append([]);
                for idxNum, goldenNum in enumerate(goldenRow):
                    try:
                        outputArray[-1].append(
                            (goldenNum - targetArray[idxRow][idxNum])/goldenNum
                        )
                    except ZeroDivisionError:
                        outputArray[-1].append('DivByZero')



except IOError as e:
    print(e)
'''


        data = csv.reader(csvfile, delimiter=',', quotechar='\n')

        for row in data:
            dataArray.append([]);
            for e in row:
                dataArray[-1].append(toFloat(e))

        lastdataArray = [];
        try:
            with open('final.csv', 'rb') as lastfile:
                data = csv.reader(lastfile, delimiter=',', quotechar='\n')

                for row in data:
                    for e in row:
                        lastdataArray.append(toFloat(e))

                for idx, last in enumerate(lastdataArray):
                    try:
                        print (dataArray[-1][idx] - last)/dataArray[-1][idx]
                    except ZeroDivisionError:
                        print 'Divided by zero'
        except IOError:
            print 'No final.csv, assuming starting for the first time...'

        with open('final.csv', 'wb') as output:
            writer = csv.writer(output, delimiter=',', quotechar='\n')
            writer.writerow(dataArray[-1])
except IOError:
    print 'No log_rocket_csv.csv, please run input.py with rocket_csv.dr at least once.'
'''
