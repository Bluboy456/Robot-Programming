import csv
import numpy as np
import calendar

list(calendar.day_name)  #  create a list of days of the week, starting monday

#read temperature csv file as a 2D list and convert to array
with open('lincoln_weather.csv', 'rb') as csvfile:
    weather_csv = csv.reader(csvfile, delimiter=',')
    weather_array = np.array(list(weather_csv))
    weather_array = np.delete(weather_array, (0), axis=0)  #remove header row 
    weather_array = weather_array.astype(float)  #csv reads in as chars, convert to int

#create a 3 x 7 array to hold frequency table - frequency[weather_type][day]
frequency = np.zeros((3,7),dtype= float)   #float so that array can be divided to produce probablity array

# fill the frequency table
for day in range(7):
    for week in range(52):  
        for weather_type in range(3):
            #print (day, week, weather_type) #DEBUG
            if weather_array[week,day] == (weather_type+1):
                #print 'match found'  #DEBUG
                frequency[weather_type][day] += 1

#create a 3 element array storing probabity of each weather_type per day (days with that weather/total number of days):
weather_probability = np.sum(frequency, 1, dtype=float)/(52*7)  #sums across rows (days)

#calculate expected weather for each day in 7 element array
expected_weather = np.sum(weather_array, 0)/52

#divide frequencies by weeks in a year to get a 3 x 7 arrary of P(weather|day)
probability_weather_on_given_day = frequency/52



# calculate 3 x7 array of P(day|weather) using Bayes rule 
# = P(weather|day)*P(day) / (P(weather)
# P(day) = 1/7
probability_of_day_if_rainy = probability_weather_on_given_day[0]/7/weather_probability[0]
probability_of_day_if_misty = probability_weather_on_given_day[1]/7/weather_probability[1]
probability_of_day_if_sunny = probability_weather_on_given_day[2]/7/weather_probability[2]


# ANSWER QUESTIONS

# Q1: Probability of being sunny at the weekend = (number of sunny Saturdays + number of sunny Sundays)/(number of Saturdays and Sundays)
prob_sunny_weekend = probability_weather_on_given_day[2,5]+probability_weather_on_given_day[2,6]
print('\n' + '\n' + 'Probability of being sunny at the weekend = (number of sunny Saturdays + number of sunny Sundays)/(52*2) = ' + 
"{:.2f}".format((frequency[2,5]+frequency[2,6])/104)  + "\n" )

# Q2: Expected weather for each day of the week
for i in range(7):
    print('Expected weather for ' + calendar.day_name[i] + ' is:' + "{:.2f}".format(expected_weather[i]))

#Q3: Most likely weather given day

print( '\n' + 'The probabilties of it raining on a given day are:')
for i in range(7):
    print(calendar.day_name[i] + ':' + "{:.2f}".format(probability_weather_on_given_day[0,i]))

print( '\n' + 'The probabilties of it being misty on a given day are:')
for i in range(7):
    print(calendar.day_name[i] + ':' + "{:.2f}".format(probability_weather_on_given_day[1,i]))

print('\n' +  'The probabilties of it being sunny on a given day are:')
for i in range(7):
    print(calendar.day_name[i] + ':' + "{:.2f}".format(probability_weather_on_given_day[2,i]))


#find most likely rainy, misty and sunny days
max_elements = np.argmax(probability_weather_on_given_day, axis=1)
print('\n' + 'Looking at the highest probabilty for each type of weather:')
print('If it is raining it is more likely to be ' + calendar.day_name[max_elements[0]])
print('If it is misty it is more likely to be ' + calendar.day_name[max_elements[1]])
print('If it is sunny it is more likely to be ' + calendar.day_name[max_elements[2]])