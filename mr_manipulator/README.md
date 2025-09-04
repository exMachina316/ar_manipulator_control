# mr_manipulator
A mixed reality whiteboard and a robot manipulator copying movements

# Steps to create virtual enviornment (Only Required once)
1. Navigate to `sign_language_model` directory in the terminal. Click here for [tutorial](https://youtu.be/Vt9WzriuSf0?si=Gi1Z94rdIuxJ5w63&t=50)

2. Install `virtualenv` tool. Follow the windows [tutorial](https://www.geeksforgeeks.org/creating-python-virtual-environment-windows-linux/) for rest of the steps
    ```sh
    pip install virtualenv
    ```

3. Create a virtual enviornment
    ```sh
    python3 -m venv .venv
    ```

4. Activate the virtual environment
    ```sh
    .venv\Scripts\activate
    ```

5. Install dependencies
    ```sh
    pip3 install -r requirements.txt
    ```

# Run your files
To run any python program with this virtual enviornment
1. Make sure virtual enviornment is active (Step 4).
2. Run your file:
    ```sh
    python3 <filename>.py
    ```