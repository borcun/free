from flask import Flask, request, jsonify

app = Flask(__name__)

@app.route('/netcorg-server', methods=['POST'])
def receive_json():
    try:
        # Parse the JSON data
        data = request.get_json()
        if not data:
            return jsonify({"error": "No JSON data received"}), 400
        
        # Log the received data
        print("Received JSON data:", data)
        
        # Process the data (example response)
        response = {"message": "Data received successfully", "data": data}
        return jsonify(response), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    app.run(host='127.0.0.1', port=5000, debug=True)
