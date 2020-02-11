from Webwork_bot import *
import itertools
import time
from login_info import j_pass, j_user

# A3 url
a3_url = 'https://webwork.math.mcgill.ca/webwork2/MATH240_WINTER2020/Assignment_3/7/'
answers = ['A', 'B', 'C', 'D', 'E']
possible_answer_list = itertools.permutations(answers)

# A4
a4_url = 'https://webwork.math.mcgill.ca/webwork2/MATH240_WINTER2020/Assignment_4/2/'
# Question 1, 2 creating all combinations
answers = "YNY"
answers_2 = 'NYN'
possible_combinations_one = itertools.combinations_with_replacement(answers, 3)
possible_combinations_two = itertools.combinations_with_replacement(answers_2, 3)
all_combinations = []
for combination in possible_combinations_one:
    if combination not in all_combinations:
        all_combinations.append(combination)
for combination in possible_combinations_two:
    if combination not in all_combinations:
        all_combinations.append(combination)
# combining combinations from the two lists (no duplicates)
combinations_2D = []
for end_combination in all_combinations:
    listCombinations = list(end_combination)
    if ['Y','Y','Y'] == listCombinations:
        listCombinations.append('Y')
        combinations_2D.append(listCombinations)
    else:
        listCombinations.append('N')
        combinations_2D.append(listCombinations)
for combo in combinations_2D:
    print(combo)


#
#
#
#
#
#
#
run_it = False
# web work bot entering combinations A3
if run_it:
    web_work_bot = WebWorkBot()
    web_work_bot.login(a3_url, j_user, j_pass)

    sleep_time = .01
    for possible_answers in possible_answer_list:
        web_work_bot.erase_answers_A3()
        time.sleep(sleep_time)
        web_work_bot.enter_answers_A3(possible_answers)
        time.sleep(sleep_time)
        web_work_bot.submit_answers()

# run bot
run_it_A4 = True
# web work bot entering combinations A4
if run_it_A4:
    sleep_time = .2
    web_work_bot = WebWorkBot()
    web_work_bot.login(a4_url, j_user, j_pass)
    for answers_r1 in combinations_2D:
        for answers_r2 in combinations_2D:
            for answers_r3 in combinations_2D:
                for answers_r4 in combinations_2D:
                    answer_combinations_r1 = answers_r1
                    answer_combinations_r2 = answers_r2
                    answer_combination_r3 = answers_r3
                    answer_combination_r4 = answers_r4
                    web_work_bot.erase_answers_A4()
                    time.sleep(sleep_time)
                    web_work_bot.enter_answers_A4(answer_combinations_r1, answer_combinations_r2, answer_combination_r3,
                                                  answer_combination_r4)
                    time.sleep(sleep_time)
                    web_work_bot.submit_answers()
