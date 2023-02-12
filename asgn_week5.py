from typing import List
from random import randint
# Do 2 queens attack each other?
# an 8x8 array representing a chessboard

def create_board(size: int) -> List[List[int]]:
    board = []
    for i in range(size):
        row = []
        for j in range(size):
            row.append(0)
        board.append(row)
    return board

def print_board(board: List[List[int]]):
    for i in range(len(board)):
        for j in range(len(board[i])):
            print(board[i][j], end=" ")
        print()
    print()
    
def add_random_queen(chess_board: List[List[int]]) -> List[int]:
    # place a queen randomly on the board
    random_indexes = (randint(0, len(chess_board) - 1), randint(0, len(chess_board) - 1)) # a tuple of indexes randomly choosen
    if chess_board[random_indexes[0]][random_indexes[1]] == 1:
        print("Can't insert queen randomly, position is already taken, rerunning 'add_random_queen'")
        add_random_queen(chess_board)
    try:
        chess_board[random_indexes[0]][random_indexes[1]] = 1
    except IndexError:
        print("Can't insert queen randomly")
    return chess_board

def insert_queen(chess_board: List[List[int]], row: int, col: int) -> bool:
    try:
        chess_board[row][col] = 1
        return True
    except IndexError:
        print("Row or Col to insert is out of bounds on board")
    return False

def number_of_queens(board: List[List[int]]) -> int:
    '''
    @Param board: list of list of integers representing the 8x8 chess board with exactly two queens in it
    Returns the number of queens on the board
    '''
    count = 0
    for i in range(len(board)):
        for j in range(len(board[0])):
            if board[i][j] == 1:
                count += 1
    return count

def isOutsideBorder(board: List[List[int]], row: int, col: int) -> bool:
    '''
    @Param board: list of list of integers representing the 8x8 chess board with exactly two queens in it
    @Param row: row index of the queen to check
    @Param col: column index of the queen to check
    Returns True if the row, col indexes are outside the board borders, False otherwise
    '''
    if row < 0 or row >= len(board):
        return True
    if col < 0 or col >= len(board[0]):
        return True
    return False


def first_queen(board: List[List[int]]) -> List[int]:
    '''
    @Param chess_board: list of list of integers representing the 8x8 chess board with exactly two queens in it
    Returns a List[int], in which the elements of the list are the indexes of the row and column for the first found queen
    The function finds the first queen by traversing the chessboard from top to bottom and from left to right 
    Then returns a list[int] containing the indexes (row in the index 0 and the column in the index 1) of first queen found
    '''
    # traversing the rows
    for i in range(len(board)):
        # traversing the columns
        for j in range(len(board[0])):
            # checking if there is a queen in that cell
            if board[i][j] == 1:
                # found queen, return position and do not continue looping
                return [i, j]
    # no queen found -> return something to handle the error in this case [-1,-1]
    return [-1,-1] # error

def queens(board: List[List[int]], verbose = False) -> bool:
    '''
    return True iff the two queens in board attack each other
    prereq1: there are two queens in board
    prereq2: len(board) == 8
    prereq3: len(board[j]) == 8
    '''
    # check the prerequisites
    # make sure number of row is equal to 8
    if len(board)!= 8:
        print("There are not exactly 8 row on the board")
        return False
    # make sure each row is length of 8 => there are 8 columns
    for i in range(len(board)):
        if len(board[i]) != 8:
            print("Each column does not have a length of 8")
            return False
    # make sure that there are ONLY 2 queens on the board
    if number_of_queens(board)!= 2:
        print("Theres not exactly two queens located on the board")
        return False
    
    # now check if queens can attack each other
    first_queen_pos = first_queen(board) # search for first queen index
    first_queen_row = first_queen_pos[0]
    first_queen_col = first_queen_pos[1]
    # check all available moves of first queen
    # if 2nd queen is located in its attack reach, then return True
    # if 2nd queen is not located in its attack reach, then return False

    # iterate through all moves, if chessboard[i][j] == 1 -> then return true as first queen can attack it 
    # else return 0 as no matter where the 2nd queen is located, the first queen can't attack it

    # find all horizontal moves and check if 2nd queen is located
    # iterating through the row where first queen is found
    for j in range(len(board[first_queen_row])):
        if board[first_queen_row][j] == 1:
            if j != first_queen_col:
                if board[first_queen_row][j] == 1:
                    if verbose:
                        print(f'- 1st Queen can attack 2nd Queen at [{first_queen_row},{j}] horizontally')
                    return True

    # find all vertical moves
    for i in range(len(board)):
        if board[i][first_queen_col] == 1:
            if i!= first_queen_row:
                if board[i][first_queen_col] == 1:
                    if verbose:
                        print(f'- 1st Queen can attack 2nd Queen at [{i},{first_queen_col}] vertically')
                    return True

    # find all diagonal moves
    # a map of diagonal directions
    diagonal_map = [[-1, -1], [-1, +1], [+1, -1], [+1, +1]] # these are four direction vectors
    # iterate through all diagonal moves
    for i in range(len(diagonal_map)):
        move_row = diagonal_map[i][0]
        move_col = diagonal_map[i][1]
        # iterate through each diagonal until we hit a border
        row = first_queen_row
        col = first_queen_col
        while True:
            # increment in diagonal direction and check if 2nd queen is located
            row = row + move_row
            col = col + move_col
            # if we are out of the border then break out otherwise we'll get an index error
            # by breaking we are going to check the next diagonal direction
            if isOutsideBorder(board, row, col):
                break

            if board[row][col] == 1:
                if board[row][col] == 1:
                    if verbose:
                        print(f'- 1st Queen can attack 2nd Queen at [{row},{col}] diagonally')
                    return True
            


    # else as we iterated through all possible moves; first queen can not attack second queen return false
    return False


if __name__ == "__main__":
    chess_board = create_board(8)
    # place two queens randomly on the board
    add_random_queen(chess_board)
    add_random_queen(chess_board)
    print("A Board with 2 queens inserted randomly:")
    print_board(chess_board)
    print('Question 1:')
    print(f'First Queen found at {first_queen(chess_board)} in chess_board')
    print('Question 2:')
    canAttack = queens(chess_board)
    # testing different boards
    print(f'First Queen can attack 2nd queens -> {canAttack}\n')

    # try different boards
    print("Test queens() on a board where 2 queens are horizontal:")
    queens_horizontal_board = create_board(8)
    insert_queen(queens_horizontal_board, 0, 4)
    insert_queen(queens_horizontal_board, 0, 7)
    print(queens(queens_horizontal_board, verbose=True))

    print()
    print("Test queens() on a board where 2 queens are vertical:")
    queens_vertical_board = create_board(8)
    insert_queen(queens_vertical_board, 1, 2)
    insert_queen(queens_vertical_board, 4, 2)
    print(queens(queens_vertical_board, verbose=True))


    print()
    print("Test queens() on a board where 2 queens are diagonal:")
    queens_diagonal_board = create_board(8)
    insert_queen(queens_diagonal_board, 3, 3)
    insert_queen(queens_diagonal_board, 7, 7)
    print(queens(queens_diagonal_board, verbose=True))

    print()
    print("Doing tests from assignment pdf")
    print("Q1:")
    board_a = create_board(8)
    insert_queen(board_a, 0, 2)
    insert_queen(board_a, 3, 4)
    board_b = create_board(8)
    insert_queen(board_b, 2, 4)
    insert_queen(board_b, 5, 1)
    board_c = create_board(8)
    insert_queen(board_c, 2, 4)
    insert_queen(board_c, 6, 4)
    board_d = create_board(8) 
    insert_queen(board_d, 6, 7)
    insert_queen(board_d, 7, 0)
    # test first_queen() 
    print(first_queen(board_a)) # [0,2]
    print(first_queen(board_b)) # [2,4]
    print(first_queen(board_c)) # [2,4]
    print(first_queen(board_d)) # [6,7]

    print("Q2:")
    print(queens(board_a))
    print(queens(board_b))
    print(queens(board_c))
    print(queens(board_d))



