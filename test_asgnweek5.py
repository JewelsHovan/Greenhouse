from asgn_week5 import *

# creating the board
aBoard = create_board(8)
print_board(aBoard)

# inserting a queen
insert_queen(aBoard, 1, 1) # inserts a queen on 2nd column and 2nd row
print_board(aBoard)
add_random_queen(aBoard) # inserts a queen randomly
print_board(aBoard)


# traversing the board
# with a simple for loop we can loop through each row like this
#for i in range(len(aBoard)):
#    print(aBoard[i])

# now that we can iterate through each row, within each row we can iterate through each column (left to right direciton)
#aRow = aBoard[0] #the first row of board
#for j in range(len(aRow)):
#    print(aRow[j])


# test the first_queens function
first_queen_position = first_queen(aBoard)
print(first_queen_position)