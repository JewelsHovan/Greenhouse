#include <iostream>
#include <string.h>
#include <fstream>
#include <sstream>

using namespace std;

/**
 * A class representing a node of a linked list
*/
class Node {
public:
    int version_id; // an integer representing the version id of the node
    int hash_value; // an integer representing the hash value of the node
    string content; // a string representing the content of the node
    Node* next; // a pointer to the next node in the list

    /**
     * A constructor to initialize the Node object
     * @param pVersion_id - integer value representing the version id
     * @param pHash_value - integer value representing the hash value of the node
     * @param pContent - a string representing the text content of the node
    */
    Node(int pVersion_id, int pHash_value, string pContent) {
        version_id = pVersion_id;
        hash_value = pHash_value;
        content = pContent;
        next = nullptr;
    }

    /**
     * A method to print the attributes of the Node object
    */
    void print(){
        std::cout << "Version number: " << this->version_id << std::endl;
        std::cout << "Hash value: " << this->hash_value << std::endl;
        std::cout << "Content: " << this->content << std::endl;
        std::cout << "\n";
    }
};

class LinkedList {
public:
    /**
     * a constructor to initialize the LinkedList object
     * Initializes an empty linked list with a null head.
     */
    LinkedList() {
        head = nullptr;
    }

    /**
     * Inserts a new Node object into the linked list
     * If the linked list is empty, newNode becomes the head
     * Otherwise newNode is appended to end of the linked list
     * current_version is set to newNode
     * @param newNode - a pointer to Node object to add to Linked List
     * @post current_version = newNode
    */
    void insert(Node* newNode) {
        if (head == nullptr) {
            head = newNode;
        } else {
            Node* temp = head;
            while (temp->next != nullptr) {
                temp = temp->next;
            }
            temp->next = newNode;
        }
        current_version = newNode;
    }

    /**
     * Removes a Node object with a matching version_id from the linked list
     * Returns true if a Node object was removed and false otherwise
     * @param version_id - an integer of the version_id attribute of the Node object to remove from the linked list
    */
    bool removeNodeById(int version_id) {
        Node *currentNode = head;
        Node *previousNode = nullptr;

        while(currentNode != nullptr) { // traverse linked list
            if(currentNode->version_id == version_id) { // compare by version_id
                // maintain LL properties
                if(currentNode == head) {
                    head = currentNode->next;
                } else {
                    previousNode->next = currentNode->next;
                }
                delete currentNode;
                return true;
            }
            previousNode = currentNode;
            currentNode = currentNode->next;
        }
        return false;
    }

    /**
     * Searches for and returns a Node object with matching version_id from the linked list
     * Returns nullptr if a Node object with matching version_id is not found
     * @param version_id - an integer of the version_id attribute of the Node object to search for
    */
    Node* getVersionId(int version_id){
        Node* temp = head;
        while (temp!= nullptr) {
            if (temp->version_id == version_id) {
                return temp;
            }
            temp = temp->next;
        }
        return nullptr;
    }

    /**
     * Iterates through the linked list and deletes each Node object from memory
    */
   void freeUp(){
        Node* currentNode = head;
        Node* temp = nullptr;
        while(currentNode != nullptr){
            temp = currentNode;
            currentNode = currentNode->next;
            delete(temp);
        }
   }

    /**
     * Returns the number of Node objects currently stored in linked list
    */
    int getSize(){
        Node* temp = head;
        int count = 0;
        while (temp!= nullptr) {
            count++;
            temp = temp->next;
        }
        return count;
    }

    /**
     * Prints out the version_id, hash_value, and content fields of each Node object 
     * stored in the linked list
    */
    void print() {
        Node* temp = head;
        std::cout << "Number of versions: " << this->getSize() << std::endl;
        while (temp != nullptr) {
            temp->print();
            temp = temp->next;
        }
        std::cout << std::endl;
    }

    // setters and getters
    Node* getCurrentVersion(){return current_version;}
    void setCurrentVersion(Node* pNode){current_version = pNode;}
    Node* getHead(){return head;}

private:
    Node* head; // Node pointer representing the head of the linked list
    Node* current_version; // Node pointer representing the current version of the text editor
};

/**
 * A class representing a git implementation of a file tracker. It tracks the file "file.txt"
 * and will keep all versions of this file in memory. 
 * The user decides when to take a snapshot of his/her modification. 
 * The user should be able to load a specific version to make it the current version or can also decide to delete any version of the file
*/
class git322{
public:
    /**
     * git322 class constructor 
     * Begins the welcome prompt with user options
     * And Starts the program and waits until user enters feature options
    */
    git322(){
        // welcome prompt
        welcome();
        startProgram();
    }

private:
    // fields
    LinkedList* ll; // a linked list object representing the list of versions git322 tracks

    // private methods
    void welcome(){
        std::cout << "Welcome to the Comp322 file versioning system!" << std::endl;
        std::cout << "\nTo add the content of your file to version control press 'a'" << std::endl;
        std::cout << "To remove a version press 'r'" << std::endl;
        std::cout << "To load a version press 'l'" << std::endl;
        std::cout << "To print to the screen the detailed list of all versions press 'p'" << std::endl;
        std::cout << "To compare any 2 versions press 'c'" << std::endl;
        std::cout << "To search versions for a keyword press 's'" << std::endl;
        std::cout << "To exit press 'e'" << std::endl;
    }

    void startProgram(){
        // initialize the linked list
        ll = new LinkedList();
        // variables 
        char key_pressed;
        bool will_exit = false;
        std::fstream file_ptr;
        std::string content;
        string version_id;
        string compare_version_one;
        string compare_version_two;
        string search_keyword;
        string version_to_delete;

        // loop that handles user input and leaves behavior implementation in add/remove/load/compare/search methods
        while(!will_exit){
            std::cin >> key_pressed;
            switch(key_pressed){
                case 'a': // Add case
                    // open and read file content
                    file_ptr = openFile("file.txt"); // throws error if failed to open
                    content = readFile(file_ptr);
                    add(content);
                    file_ptr.close(); // close file
                    break;
                case 'r': // Remove case
                    std::cout << "Enter the number of the version that you want to delete:" << std::endl;
                    std::cin >> version_to_delete;
                    remove(std::stoi(version_to_delete));
                    break;
                case 'l': // Load Case
                    std::cout << "Which version would you like to load?" << std::endl;
                    std::cin >> version_id;
                    load(std::stoi(version_id));
                    break;
                case 'p': // print case
                    // iterate through ll and print each node
                    ll->print();
                    break;
                case 'c': // Compare case
                    std::cout << "Please enter the number of the first version to compare:" << std::endl;
                    std::cin >> compare_version_one;
                    std::cout << "Please enter the number of the second version to compare:" << std::endl;
                    std::cin >> compare_version_two;
                    compare(std::stoi(compare_version_one), std::stoi(compare_version_two));
                    break;
                case 's': // Search case
                    std::cout << "Please enter the keyword you are looking for:" << std::endl;
                    std::cin >> search_keyword;
                    search(search_keyword);
                    break;
                case 'e': // Exit case
                    std::cout << "Exiting program..." << std::endl;
                    will_exit = true;
                    ll->freeUp(); // delete each Node object
                    delete ll;
                    break;
            }
        }
    }

    /**
     * Adds the current version of the text file to the linked list
     * If the current version is unmodified/unchanged from previous saved version it outputs to user 
     * @param string content - a string representing the current versions text to add to the linked list of versions
    */
    void add(string content){
        // check if content is same
        if(ll->getSize() > 0){
            if(hash_it(ll->getCurrentVersion()->content) == hash_it(content)){
                std::cout << "git322 did not detect any change to your file and will not create a new version." << std::endl;
                return;
            }
        }
        // create and add new node to linked list 
        int version_id = ll->getSize() + 1;
        int hash_value = hash_it(content);
        Node* newNode = new Node(version_id, hash_value, content);
        ll->insert(newNode);
        std::cout << "Your content has been added successfully!" << std::endl;
    }
    /**
     * Prints out all the versions saved in the linked list
    */
    void print(void){
        ll->print();
    }
    /**
     * Loads a version saved in the linked list and overwrites the text file of the content loaded
     * @param int version - integer value of the version number to overwrite the current text file
     * @post ll->current_version = rtr_node
    */
    void load(int version){
        string content_to_load;
        Node* rtr_node = ll->getVersionId(version); // node pointer of Node object with the version id searched for
        if(rtr_node != nullptr){
            content_to_load = rtr_node->content;
            overwriteFile("file.txt", content_to_load);
            ll->setCurrentVersion(rtr_node);
            std::cout << "Version " << version << " loaded successfully. Please refresh your text editor to see the changes." << std::endl;
        }
        else{
            std::cout << "Please enter a valid version number. If you are not sure please press 'p' to list all valid version numbers." << std::endl;
        }

    }
    /**
     * Compares two Node objects of the Linked list 
     * @param version1 - integer value of the version id of the first node to compare
     * @param version2 - integer value of the version id of the second node to compare
    */
    void compare(int version1, int version2){
        Node* node_one = ll->getVersionId(version1);
        Node* node_two = ll->getVersionId(version2);
        compareStrings(node_one->content, node_two->content);
    }
    /**
     * Given a keyword as a string, searches for all Node objects of Linked list with the keyword in the content field
     * @param keyword - string value representing the user entered keyword to search for 
    */
    void search(string keyword){
        Node* temp = ll->getHead();
        bool foundAny = false;
        while(temp != nullptr){
            if(isKeywordInContent(keyword, temp->content)){
                if(!foundAny){
                    std::cout << "The keyword '" << keyword << "' has been found in the following versions:" << std::endl;
                    foundAny = true;
                }
                temp->print();
            }
            temp = temp->next;
        }
        if(!foundAny){
            std::cout << "Your keyword " << keyword << " was not found in any version." << std::endl;
        }
    }       
    /**
     * Given a version_id, removes the Node object in Linked list matching with the version id
     * @param version - integer value representing the version_id matching the Node to remove
     * @post The linked list is intact
    */
    void remove(int version){
        bool removed = ll->removeNodeById(version);
        if(removed){
            std::cout << "Version " << version << " was removed successfully." << std::endl;
        }
        else{
            std::cout << "Please enter a valid version number." << std::endl;
        }
    }

    // helper methods
    std::fstream openFile(const std::string& filename) {
        /*
        Given a filename as a string, opens a file for reading and writing 
        Throws an error if file failed to open
        Returns a std::fstream pointer 
        */
        std::fstream file(filename, std::ios::in | std::ios::out);
        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << filename << std::endl;
        }
        return file;
    }

    std::string readFile(std::fstream& file) {
        /*
        Given a file pointer as a std::fstream object, reads the contents of the file and returns it as a string
        */
        std::stringstream buffer;
        buffer << file.rdbuf();

        return buffer.str();
    }

    void overwriteFile(const string& filename, const string& content) {
        /*
        Given the filename and content as strings, overwrites the file matching with the filename with the content
        */
        std::ofstream ofs(filename);
        ofs << content;
        ofs.close();
    }

    void compareStrings(const string& str1, const string& str2) {
        /*
        Given two string objects, compares the two strings line by line
        */
        // convert into stream objects, to iterate through lines
        std::stringstream ss1(str1);
        std::stringstream ss2(str2);

        std::string line1, line2;
        // Line count and identical variables
        int lineNum = 1;
        bool identical = true; // if strings are identical 

        // for lines matching between str1 and str2
        while (std::getline(ss1, line1) && std::getline(ss2, line2)) {
            if (line1 == line2) {
                std::cout << "Line " << lineNum << ": <Identical>" << std::endl;
            } else {
                identical = false;
                std::cout << "Line " << lineNum << ": " << line1 << " <<>> " << line2 << std::endl;
            }
            lineNum++;
        }

        // handle any remaining lines in the remaining longer string
        // if longer is str1
        while (std::getline(ss1, line1)) {
            identical = false;
            std::cout << "Line " << lineNum << ": " << line1 << " <<>> <Empty line>" << std::endl;
            lineNum++;
        }

        // if longer is str2
        while (std::getline(ss2, line2)) {
            identical = false;
            std::cout << "Line " << lineNum << ": <Empty line> <<>> " << line2 << std::endl;
            lineNum++;
        }
    }

    /**
     * Given a keyword as a string, searches for all Node objects of Linked list with the keyword in the content field
     * @param keyword - string value representing the user entered keyword to search for
     * @param content - string content of a Node object 
     * @returns a boolean, true if keyword is found in content, false otherwise
    */
    bool isKeywordInContent(std::string keyword, std::string content) {
        return (content.find(keyword) != std::string::npos); // npos is returned if find() does not return an index integer
    }

    std::size_t hash_it (std::string someString){
        /*
        Return the hash value of a string 
        */
        std::size_t hash_value = hash<string>{}(someString);
        return hash_value;
    }
};

int main(int argc, char *argv[]){
    // initialize and run the tracking program
    git322 git;
    return 0;
}