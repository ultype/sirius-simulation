import csv

def toFloat(str):
    try:
        return float(str)
    except ValueError:
        return str

dataArray = [];

try:
    with open('log_rocket_csv.csv', 'rb') as csvfile:
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
