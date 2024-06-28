# app.py
import os
from ament_index_python.packages import get_package_share_directory
from flask import Flask, request, jsonify, send_file

app = Flask(__name__)

@app.route('/', methods=['GET'])
def hello():
    pkg_path = os.path.join(get_package_share_directory('my_package'))
    index_path = os.path.join(pkg_path, 'templates/index.html')
    return send_file(index_path)

@app.route('/system_status', methods=['POST'])
def system_status():
    data = request.get_json()
    print(f"Received system status: {data['status']}")
    return jsonify({"response": "Status received"}), 200

def main(arg=None):
    app.run(debug=False)

if __name__ == '__main__':
    main()
