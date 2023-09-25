FROM python:3.11-slim

# Install required libraries
COPY app/requirements.txt /requirements.txt
RUN pip3 install -r requirements.txt

# Copy your Python monitoring script into the container
COPY app /app