# Vorto Algorithm Challenge
 
Create a virtual environment in python and activate it using these commands:
```sh
python3 -m venv venv
source venv/bin/activate (or venv\Scripts\activate if you are using Windows)
```

Install dependencies in virtual environment:
```sh
pip3 install -r requirements.txt
```

Running solution:
```sh
py mySubmission.py "./Training Problems/problem1.txt"
```

Running Test:
```sh
py evaluateShared.py "./Training Problems" "py mySubmission.py"
```
#### Note: mySubmission.py must be tested against this version of evaluateShared.py. This is because, original evaluateShared encoutered an error while parsing STDOUT even though mySubmission file produces final output as expected. Line number 78 in this version of evaluateShared.py replaces /r with an empty string ("") following which the tests run as expected. Other than added code at line 78, the file content is same.
