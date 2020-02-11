from selenium import webdriver
import itertools
from bs4 import BeautifulSoup


class WebWorkBot:
    """
    Bot that will post answers to web work
    """
    current_url = ''

    def __init__(self):
        self.driver = webdriver.Chrome(executable_path=r'C:/webdrivers/chromedriver.exe')

    def login(self, url, username, password):
        self.driver.get(url)
        # finding xpath to login
        user_log = self.driver.find_element_by_xpath('//*[@id="uname"]')
        pass_log = self.driver.find_element_by_xpath('//*[@id="pswd"]')
        continue_click = self.driver.find_element_by_xpath('//*[@id="none"]')
        # entering info and continue
        user_log.send_keys(username)
        pass_log.send_keys(password)
        continue_click.click()
        # set field
        self.current_url = url

    # For Assignment 3
    def enter_answers_A3(self, answer_tuple):
        # finding xpaths
        q_one = self.driver.find_element_by_xpath('//*[@id="AnSwEr0001"]')
        q_two = self.driver.find_element_by_xpath('//*[@id="AnSwEr0002"]')
        q_three = self.driver.find_element_by_xpath('//*[@id="AnSwEr0003"]')
        q_four = self.driver.find_element_by_xpath('//*[@id="AnSwEr0004"]')
        q_five = self.driver.find_element_by_xpath('//*[@id="AnSwEr0005"]')
        # entering answers
        a_one, a_two, a_three, a_four, a_five = answer_tuple
        q_one.send_keys(a_one)
        q_two.send_keys(a_two)
        q_three.send_keys(a_three)
        q_four.send_keys(a_four)
        q_five.send_keys(a_five)

    def erase_answers_A3(self):
        q_one = self.driver.find_element_by_xpath('//*[@id="AnSwEr0001"]')
        q_two = self.driver.find_element_by_xpath('//*[@id="AnSwEr0002"]')
        q_three = self.driver.find_element_by_xpath('//*[@id="AnSwEr0003"]')
        q_four = self.driver.find_element_by_xpath('//*[@id="AnSwEr0004"]')
        q_five = self.driver.find_element_by_xpath('//*[@id="AnSwEr0005"]')
        q_list = [q_one, q_two, q_three, q_four, q_five]
        for question in q_list:
            question.clear()

    # For assignment 4
    def enter_answers_A4(self, answers_r1, answers_r2, answers_r3, answers_r4):
        # finding xpaths
        r1c1 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0001"]')
        r1c2 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0002"]')
        r1c3 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0003"]')
        r1c4 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0004"]')
        r2c1 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0005"]')
        r2c2 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0006"]')
        r2c3 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0007"]')
        r2c4 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0008"]')
        r3c1 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0009"]')
        r3c2 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0010"]')
        r3c3 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0011"]')
        r3c4 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0012"]')
        r4c1 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0013"]')
        r4c2 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0014"]')
        r4c3 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0015"]')
        r4c4 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0016"]')
        # enter information
        # row 1
        r1c1_answers, r1c2_answers, r1c3_answers, r1c4_answers = answers_r1
        # row 2
        r2c1_answers, r2c2_answers, r2c3_answers, r2c4_answers = answers_r2
        # row 3
        r3c1_answers, r3c2_answers, r3c3_answers, r3c4_answers = answers_r3
        # row 4
        r4c1_answers, r4c2_answers, r4c3_answers, r4c4_answers = answers_r4
        # enter answers
        r1c1.send_keys(r1c1_answers)
        r1c2.send_keys(r1c2_answers)
        r1c3.send_keys(r1c3_answers)
        r1c4.send_keys(r1c4_answers)
        r2c1.send_keys(r2c1_answers)
        r2c2.send_keys(r2c2_answers)
        r2c3.send_keys(r2c3_answers)
        r2c4.send_keys(r2c4_answers)
        r3c1.send_keys(r3c1_answers)
        r3c2.send_keys(r3c2_answers)
        r3c3.send_keys(r3c3_answers)
        r3c4.send_keys(r3c4_answers)
        r4c1.send_keys(r4c1_answers)
        r4c2.send_keys(r4c2_answers)
        r4c3.send_keys(r4c3_answers)
        r4c4.send_keys(r4c4_answers)

    def erase_answers_A4(self):
        r1c1 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0001"]')
        r1c2 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0002"]')
        r1c3 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0003"]')
        r1c4 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0004"]')
        r2c1 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0005"]')
        r2c2 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0006"]')
        r2c3 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0007"]')
        r2c4 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0008"]')
        r3c1 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0009"]')
        r3c2 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0010"]')
        r3c3 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0011"]')
        r3c4 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0012"]')
        r4c1 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0013"]')
        r4c2 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0014"]')
        r4c3 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0015"]')
        r4c4 = self.driver.find_element_by_xpath('//*[@id="AnSwEr0016"]')
        q_list = [r1c1, r1c2, r1c3, r1c4, r2c1, r2c2, r2c3, r2c4, r3c1, r3c2, r3c3, r3c4, r4c1, r4c2, r4c3, r4c4]
        for question in q_list:
            question.clear()

    def submit_answers(self):
        submit_bt = self.driver.find_element_by_xpath('//*[@id="submitAnswers_id"]')
        submit_bt.click()

    def get_page_range(self):
        """
        Given url and assignment
        """
        page_source = self.driver.page_source
        soup = BeautifulSoup(page_source)
        problem_body = soup.find('u1', class_='problem-list nav nav-list')
        page_range = len(problem_body.find_all('li'))
        return (i + 1 for i in range(page_range))

    def current_page(self):
        self.current_url = self.driver.current_url

    def next_question(self):
        new_url = self.current_url.split('/')
        current_q = self.current_url[-2]
        next_q = str(int(current_q) + 1)
        new_url[-2] = next_q
        self.driver.get('/'.join(new_url))

    def previous_question(self):
        new_url = self.current_url.split('/')
        current_q = self.current_url[-2]
        next_q = str(int(current_q) - 1)
        new_url[-2] = next_q
        self.driver.get('/'.join(new_url))

    @staticmethod
    def find_permutations(iteration_list):
        """
        returns 2D list of possible permutations from input list
        """
        return itertools.permutations(iteration_list)

    @staticmethod
    def find_combinations(iteration_list):
        """
        returns 2D list of possible combinations from input list
        """
        return itertools.combinations(iteration_list)
