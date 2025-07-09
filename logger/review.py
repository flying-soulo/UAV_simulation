from flask import Flask, render_template_string, request, redirect, url_for
import pandas as pd
import plotly.graph_objs as go
import plotly.io as pio
from plotly.subplots import make_subplots
import os

app = Flask(__name__)
UPLOAD_FOLDER = 'uploads'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>CSV Data Viewer</title>
    <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
    <style>
        body { font-family: Arial, sans-serif; margin: 40px; }
        form { margin-bottom: 30px; }
        label { display: inline-block; width: 150px; font-weight: bold; }
        select[multiple] { width: 300px; height: 150px; }
        .warn { color: red; font-weight: bold; }
    </style>
</head>
<body>
    <h2>Upload CSV File</h2>
    <form method="post" enctype="multipart/form-data" action="{{ url_for('upload_file') }}">
        <input type="file" name="file" accept=".csv" required>
        <input type="submit" value="Upload">
    </form>

    {% if columns %}
    <hr>
    <h3>Select Parameters to Plot</h3>
    <form method="post" action="{{ url_for('plot') }}">
        <input type="hidden" name="filename" value="{{ filename }}">

        <label for="x_axis">X Axis:</label>
        <select name="x_axis" required>
            {% for col in columns %}
            <option value="{{ col }}" {% if col == 'time' %}selected{% endif %}>{{ col }}</option>
            {% endfor %}
        </select><br><br>

        <label for="y_axes">Y Axes (multiple):</label>
        <select name="y_axes" multiple required>
            {% for col in columns %}
            {% if col != 'time' %}
            <option value="{{ col }}">{{ col }}</option>
            {% endif %}
            {% endfor %}
        </select><br><br>

        <label><input type="checkbox" name="multi_graph" value="yes"> Plot in separate subplots</label><br><br>

        <input type="submit" value="Generate Plot">
    </form>
    {% endif %}

    {% if warnings %}
    <div class="warn">
        <p>⚠️ Some issues occurred:</p>
        <ul>
            {% for w in warnings %}
            <li>{{ w }}</li>
            {% endfor %}
        </ul>
    </div>
    {% endif %}

    {% if plot_div %}
    <hr>
    <h3>Generated Plot</h3>
    <div id="plotly-div">{{ plot_div | safe }}</div>
    {% endif %}
</body>
</html>
"""

@app.route('/', methods=['GET'])
def index():
    return render_template_string(HTML_TEMPLATE, columns=None, filename=None, plot_div=None, warnings=[])

@app.route('/upload', methods=['POST'])
def upload_file():
    file = request.files['file']
    if file and file.filename and file.filename.endswith('.csv'):
        filepath = os.path.join(app.config['UPLOAD_FOLDER'], file.filename)
        file.save(filepath)
        df = pd.read_csv(filepath)
        columns = df.columns.tolist()
        return render_template_string(HTML_TEMPLATE, columns=columns, filename=file.filename, plot_div=None, warnings=[])
    return redirect(url_for('index'))

@app.route('/plot', methods=['POST'])
def plot():
    filename = request.form['filename']
    x_axis = request.form['x_axis']
    y_axes = request.form.getlist('y_axes')
    multi_graph = request.form.get('multi_graph') == 'yes'

    filepath = os.path.join(app.config['UPLOAD_FOLDER'], filename)
    df = pd.read_csv(filepath)
    warnings = []

    # # Try to convert x_axis to float, else parse as datetime
    # try:
    #     df[x_axis] = pd.to_numeric(df[x_axis], errors='raise')
    # except Exception:
    #     try:
    #         df[x_axis] = pd.to_datetime(df[x_axis], errors='raise')
    #     except Exception:
    #         warnings.append(f"Failed to parse X axis '{x_axis}' as numeric or datetime.")
    #         return render_template_string(HTML_TEMPLATE, columns=df.columns.tolist(), filename=filename, plot_div=None, warnings=warnings)

    try:
        df[x_axis] = pd.to_numeric(df[x_axis], errors='raise')
    except Exception:
        try:
            df[x_axis] = pd.to_datetime(df[x_axis], errors='raise')
            # Convert datetime to elapsed seconds
            df[x_axis] = (df[x_axis] - df[x_axis].iloc[0]).dt.total_seconds()
            warnings.append(f"Converted datetime X axis '{x_axis}' to elapsed seconds.")
        except Exception:
            warnings.append(f"Failed to parse X axis '{x_axis}' as numeric or datetime.")
            return render_template_string(HTML_TEMPLATE, columns=df.columns.tolist(), filename=filename, plot_div=None, warnings=warnings)

    valid_y_axes = []
    for y in y_axes:
        try:
            df[y] = pd.to_numeric(df[y], errors='raise')
            valid_y_axes.append(y)
        except Exception:
            warnings.append(f"Skipping Y axis '{y}' - not numeric.")

    if not valid_y_axes:
        warnings.append("No valid Y axes selected for plotting.")
        return render_template_string(HTML_TEMPLATE, columns=df.columns.tolist(), filename=filename, plot_div=None, warnings=warnings)

    if multi_graph:
        fig = make_subplots(rows=len(valid_y_axes), cols=1, shared_xaxes=True, vertical_spacing=0.03)
        for i, y in enumerate(valid_y_axes):
            fig.add_trace(go.Scatter(x=df[x_axis], y=df[y], mode='lines+markers', name=y), row=i + 1, col=1)
            fig.update_yaxes(title_text=y, row=i + 1, col=1)
        fig.update_layout(
            height=350 * len(valid_y_axes),
            width=1000,
            title_text="Multiple Graphs (Subplots)",
            showlegend=False,
            template='plotly_white'
        )
    else:
        fig = go.Figure()
        for y in valid_y_axes:
            fig.add_trace(go.Scatter(x=df[x_axis], y=df[y], mode='lines+markers', name=y))
        fig.update_layout(
            title=f"{', '.join(valid_y_axes)} vs {x_axis}",
            xaxis_title=x_axis,
            yaxis_title="Value",
            legend_title="Parameters",
            template="plotly_white",
            height=600,
            width=1000
        )

    plot_div = pio.to_html(fig, full_html=False)
    columns = df.columns.tolist()
    return render_template_string(HTML_TEMPLATE, columns=columns, filename=filename, plot_div=plot_div, warnings=warnings)

if __name__ == '__main__':
    app.run(debug=True)
