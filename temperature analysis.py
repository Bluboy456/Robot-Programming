import csv

#read temperature csv file as a 2D list

with open('lincoln_weather.csv', 'rb') as csvfile:
    weather_csv = csv.reader(csvfile, delimiter=',')
    weather_list = list(weather_csv)

    #print(weather_list) #DEBUG

#create a file of frequencies in frequency[day][weather_type]
#Declaring an empty 1D row list.
column = []  
#Declaring an empty 1D list.
frequency = []
#Initialize the weather type column to Zeroes.
for j in range(3):
        column.append(0)
#Append the column to each row.
for day in range(7):
    frequency.append(column)


#print(frequency)#DEBUG Print the two dimensional list.


for day in range(7):
    for week in range(53):  
        for weather_type in range(1,4):
            #print (day, week, weather_type) DEBUG
            if weather_list[week][day] == str(weather_type):
                #print 'match found'  DEBUG
                frequency[day][weather_type] += 1
print(frequency)#DEBUG 
