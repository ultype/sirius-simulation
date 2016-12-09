import csv
import argparse
import matplotlib.pyplot as plt

def toFloat(str):
    try:
        return float(str)
    except ValueError:
        return str

parser = argparse.ArgumentParser(
    description='Read some csv and output landing point.'
)
parser.add_argument('golden', help='Input the file position of golden file.')
parser.add_argument('target',
    help='Input the file position of target folder.\n\
          Default: ../MONTE_RUN_monte/,
    default='../MONTE_RUN_monte/')
parser.add_argument('number',
                    help='Input the number of input file.',
                    default='20',
                    type=int)

args = parser.parse_args()

try:
    with open(args.golden, 'rb') as goldenFile:

        goldenData = csv.reader(goldenFile, delimiter=',', quotechar='\n')

        goldenArray = [];

        #Eat all shitz in
        for row in goldenData:
            goldenArray.append([]);
            for num in row:
                goldenArray[-1].append(toFloat(num))
        goldenFinal = [];
        for num in goldenArray[-1]:
            goldenFinal.append(num)

        plotArray = [[goldenFinal[7]],[goldenFinal[8]]]
        test = 0
        for i in range(0,args.number):
            print(test)
            test += 1
            text = args.target + 'RUN_' + ('00000' + str(i))[-5:] + '/log_rocket_csv.csv'
            try:
                with open(text, 'rb') as csvFile:
                    buf = []
                    csvData = csv.reader(csvFile, delimiter=',',quotechar='\n')
                    for row in csvData:
                        buf = []
                        for num in row:
                            buf.append(num)
                    plotArray[0].append(toFloat(buf[7]))
                    plotArray[1].append(toFloat(buf[8]))
            except:
                print('Error: too many cases given.')
        print(plotArray[0])
        print(plotArray[1])
        plt.plot(plotArray[0],plotArray[1],"o")
        #plt.axis([plotArray[0][0]-1000000,plotArray[0][0]+1000000,plotArray[1][0]-1000000,plotArray[1][0]+1000000])
        plt.xlabel('x(m)')
        plt.ylabel('y(m)')
        plt.show()

except IOError as e:
    print(e)
