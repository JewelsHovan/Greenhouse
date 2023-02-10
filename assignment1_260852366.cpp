#include <iostream>
#include <string.h>
#include <cstring>
#include <cstdio>

using namespace std;


// Helper functions 
std::string getTextFileName(std::string fileName){
    /*
    Helper function to splice the relative path of a file to just the file name
    */
    std::size_t found = fileName.find_last_of("/\\");
    if(found!= std::string::npos){
        return fileName.substr(found+1);
    }
    return fileName;
}

string BooltoString(bool value){
    if(value){
        return "True";
    }else{
        return "False";
    }   
}

bool word_diff(std::string word1, std::string word2){
    /*
     * Function to compare two strings
     * returns true if strings are equal
    */
    // check if both strings are same size
    if(word1.size()!= word2.size()){
        return false;
    }
    // check if all characters are the same
    for(int i=0; i<word1.size(); i++){
        if(word1[i]!= word2[i]){
            return false;
    }}
    // return true if all chekcs are passed
    return true;
}





bool classical_file_diff(std::string file1, std::string file2){
    /*
    Function to compare two files
    returns true if contents of files are equal
    */
    char buffer[255];
    char buffer2[255];
    FILE* fp1 = fopen(file1.c_str(), "r");
    FILE* fp2 = fopen(file2.c_str(), "r");
    // check line by line 
    if(fp1){
        while(fgets(buffer, 255, fp1)){
            fgets(buffer2, 255, fp2);
            // compare
            if(!word_diff(buffer, buffer2)){
                return false;
            }
        }
        std::fclose(fp1);
        std::fclose(fp2);
    }
    else{
        std::cout << "Unable to open/read files: " << file1.c_str() << endl;
    }

    return true;
}

std::size_t hash_it (std::string someString){
    /*
    Return the hash value of a string 
    */
    std::size_t hash_value = std::hash<std::string>{}(someString);
    return hash_value;
}

bool enhanced_file_diff(std::string file1, std::string file2){
    /**
     * Function to compare two files using hash signatures
     */
    char buffer[255];
    char buffer2[255];
    FILE* fp1 = fopen(file1.c_str(), "r");
    FILE* fp2 = fopen(file2.c_str(), "r");
    if(fp1){
        while(fgets(buffer, 255, fp1)){
            fgets(buffer2, 255, fp2);
            std::size_t hash1 = hash_it(buffer);
            std::size_t hash2 = hash_it(buffer2);
            // compare line hash values 
            if(hash1 != hash2){
                return false;
            }
        }
        std::fclose(fp1);
        std::fclose(fp2);
    }
    else{
        std::cout << "Unable to open/read files: " << file1.c_str() << endl;
    }
    return true;
}

void list_mismatched_lines(std::string file1, std::string file2){
    /*
    Function to compare two text files and return mismatched lines
    */
    char buffer[255];
    char buffer2[255];
    FILE* fp1 = fopen(file1.c_str(), "r");
    FILE* fp2 = fopen(file2.c_str(), "r");
    if(fp1){
        while(fgets(buffer, 255, fp1)){
            fgets(buffer2, 255, fp2);
            std::size_t hash1 = hash_it(buffer);
            std::size_t hash2 = hash_it(buffer2);
            if(hash1 != hash2){
                std::cout << getTextFileName(file1) << ": " << buffer << endl;
                std::cout << getTextFileName(file2) << ": " << buffer2 << endl;
            }
        }
        std::fclose(fp1);
        std::fclose(fp2);
    }
    else{
        std::cout << "Unable to open/read files: " << file1.c_str() << endl;
    }
}

bool list_mismatched_words_rec(char *&word1, char *&word2, char *saveptr1, char *saveptr2, int counter, std::string file1, std::string file2){
        // tokenizes every word recursively and print mismatches
        if(word1 == NULL || word2 == NULL){
            // end recursion
            return true;
        }
        else{
            // else recurse
            //printf("word1: %s word2: %s\n", word1, word2);
            if(hash_it(word1) != hash_it(word2)){
                std::cout << getTextFileName(file1) << ": " << word1 << " (line " << counter << ")" << endl;
                std::cout << getTextFileName(file2) << ": " << word2 << " (line " << counter << ")" << endl;
            }
            // point to next word (delimiter = space char)
            word1 = strtok_r(NULL," ", &saveptr1);
            word2 = strtok_r(NULL," ", &saveptr2);
            list_mismatched_words_rec(word1, word2, saveptr1, saveptr2, counter, file1, file2);
            return false; // for proper compilation
        }
}

void list_mismatched_words(std::string file1, std::string file2){
    /*
    Compares two text files
    Lists mismatched words in each line 
    */
    char buffer1[255];
    char buffer2[255];
    FILE* fp1 = fopen(file1.c_str(), "r");
    FILE* fp2 = fopen(file2.c_str(), "r");
    int counter = 0;
    // recursive through each line
    if(fp1 && fp2){
        while(fgets(buffer1, 255, fp1) && fgets(buffer2, 255, fp2)){
            counter += 1; // increment line counter
            if(hash_it(buffer1) != hash_it(buffer2)){
                char *word1 = buffer1;
                char *word2 = buffer2;
                char *saveptr1, *saveptr2;
                word1 = strtok_r(buffer1, " ", &saveptr1);   
                word2 = strtok_r(buffer2, " ", &saveptr2);   
                list_mismatched_words_rec(word1, word2, saveptr1, saveptr2, counter, file1, file2);
            }
        }
    }
    std::fclose(fp1);
    std::fclose(fp2);
}

int main()
{
    // Q1
    // Tests for word_diff
    std::cout << "###############################" << std::endl;
    std::cout << "Testing word_diff" << std::endl;
    std::string str1 = "Hello World";
    std::string str2 = "hEllO World";
    std::string str3 = "World";
    std::string str4 = "Hello World";
    bool result1 = word_diff(str1, str2);
    bool result2 = word_diff(str1, str3);
    bool result3 = word_diff(str1, str4);
    std::cout << BooltoString(result1) << endl; // False
    std::cout << BooltoString(result2) << endl; // False
    std::cout << BooltoString(result3) << endl; // True
    std::cout << "###############################" << std::endl;

    // Q2
    // tests for classical_file_diff
    std::cout << "Testing classic_file_diff" << std::endl;
    std::string file1 = "./txt_folder/file1.txt";
    std::string file2 = "./txt_folder/file2.txt";
    bool result_q2 = classical_file_diff(file1, file2);
    std::cout << BooltoString(result_q2) << endl; // False
    std::cout << "###############################" << std::endl;

    // Q3
    // tests for hash_it 
    std::cout << "Testing hash_it" << std::endl;
    std::string mystr = "I love this assignment";
    std::size_t h1 = hash_it (mystr);
    std::cout << h1 << std::endl;
    std::cout << "###############################" << std::endl;

    // Q4
    // tests for enhanced_file_diff
    std::cout << "Testing enhanced_file_diff" << std::endl;
    bool result_q4 = enhanced_file_diff(file1, file2); // False
    std::cout << BooltoString(result_q4) << endl; // False
    std::cout << "###############################" << std::endl;

    // Q5
    // tests for list_mismatched_lines
    std::cout << "Testing list_mismatched_lines" << std::endl;
    list_mismatched_lines(file1, file2); // prints to screen mismatched lines
    std::cout << "###############################" << std::endl;
    
    // Q6
    // tests for list_mismatched_words
    std::cout << "Testing list_mismatched_words" << std::endl;
    list_mismatched_words(file1, file2); // => output enjoy and like on line 2


    return 0;
}