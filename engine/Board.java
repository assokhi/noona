package engine;
public class Board {  static char[][] board = new char[9][9];
    public static void main(String[] args) {
        initBoard();
        printBoard();
    }

    // initialize labels + pieces
    static void initBoard() {

        // fill everything with space
        for (int i = 0; i < 9; i++)
            for (int j = 0; j < 9; j++)
                board[i][j] = ' ';

        // bottom letters (files)
        char file = 'a';
        for (int j = 1; j <= 8; j++)
            board[0][j] = file++;

        // left numbers (ranks)
        for (int i = 1; i <= 8; i++)
            board[i][0] = (char) ('0' + i);

        // pieces
        char[] backRankWhite = {'R','N','B','Q','K','B','N','R'};
        char[] backRankBlack = {'r','n','b','q','k','b','n','r'};

        // place pieces
        for (int j = 1; j <= 8; j++) {
            board[1][j] = backRankWhite[j-1];
            board[2][j] = 'P';

            board[7][j] = 'p';
            board[8][j] = backRankBlack[j-1];
        }

        // empty squares
        for (int i = 3; i <= 6; i++){
            for (int j = 1; j <= 8; j++)
                board[i][j] = '.';
        }
    }

    static void printBoard() {

        for (int i = 8; i >= 0; i--) {
            for (int j = 0; j < 9; j++) {
                System.out.print(board[i][j] + " ");
            }
            System.out.println();
        }
    }
}
