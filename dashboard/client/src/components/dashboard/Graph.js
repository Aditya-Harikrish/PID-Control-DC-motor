import React, { Component } from 'react';
import { Button, Form } from "react-bootstrap";
import axios from 'axios';
import { withRouter } from "react-router-dom";
// import './navbar.css'
import PropTypes from "prop-types";
import { connect } from "react-redux";
import styles from './dashboard.module.css';

class Graph extends Component {
    constructor(props) {
        super(props);
        this.state = {
            angle: '',
            k_p: 10,
            k_i: 5,
            k_d: 0.025,
        };
        this.handleAngle = this.handleAngle.bind(this);
        this.handlekp = this.handlekp.bind(this);
        this.handleki = this.handleki.bind(this);
        this.handlekd = this.handlekd.bind(this);
        this.onSubmit = this.onSubmit.bind(this);
    }
    handleAngle(event) {
        this.setState({ angle: event.target.value });
    }

    handlekp(event) {
        this.setState({ k_p: event.target.value });
    }

    handleki(event) {
        this.setState({ k_i: event.target.value });
    }

    handlekd(event) {
        this.setState({ k_d: event.target.value });
    }
    onSubmit(event) {
        event.preventDefault();

        if (isNaN(this.state.angle) || this.state.angle < 0 || this.state.angle > 360) {
            alert("Target Angle should be an integer between 0 and 360");
            return;
        }

        if (isNaN(this.state.k_p) || this.state.k_p < 0) {
            alert("k_p should be a decimal number greater than or equal to 0");
            return;
        }
        if (isNaN(this.state.k_i) || this.state.k_i < 0) {
            alert("k_i should be a decimal number greater than or equal to 0");
            return;
        }
        if (isNaN(this.state.k_d) || this.state.k_d < 0) {
            alert("k_d should be a decimal number greater than or equal to 0");
            return;
        }

        if (this.state.angle == "")
            this.setState({ angle: 0 });
        if (this.state.k_p == "")
            this.setState({ k_p: 10 });
        if (this.state.k_i == "")
            this.setState({ k_i: 5 });
        if (this.state.k_d == "")
            this.setState({ k_d: 0.025 });


        const url = 'https://api.thingspeak.com/update?api_key=S6W8OO70L5TBDX50&field1=' + this.state.angle + '&field2=' + this.state.k_p + '&field3=' + this.state.k_d + '&field4=' + this.state.k_i;
        // const url1='https://api.thingspeak.com/channels/1856865/feeds.json'
        // var requestOptions1 = {
        //     method: 'DELETE',
        //     redirect: 'follow'
        // };
        // fetch(url, requestOptions)
        //     .then(response => response.text())
        //     .then(result => console.log(result))
        //     .catch(error => console.log('error', error));
        var requestOptions = {
            method: 'DELETE',
            redirect: 'follow'
        };
        var requestOptions1 = {
            method: 'POST',
            redirect: 'follow'
        };
        fetch("https://api.thingspeak.com/channels/1856865/feeds.json?api_key=CMGTYVOSQGX3XCHI", requestOptions)
            .then(response => response.text())
            .then(result => console.log(result))
            .catch(error => console.log('error', error));
        setTimeout(() => {
            fetch(url, requestOptions1)
                .then(response => response.text())
                .then(result => console.log(result))
                .catch(error => console.log('error', error));
        }, 15500);


        // delay of 3 seconds
        setTimeout(() => { window.location.reload();}, 75000);
    }

    render() {
        return (
            <div className="col s12 center-align">
                <div>
                    <br></br>
                    <br></br>
                    <br></br>
                    <form onSubmit={this.onSubmit}>
                        <table className={styles.table}>
                            <tbody>
                                <tr className={styles.tr}>
                                    <td className={styles.td}>
                                        <label className={styles.label}>Angle (&deg;) :</label>
                                        <input type="text" value={this.state.angle} onChange={this.handleAngle} />
                                    </td>
                                    <td className={styles.td}>
                                        <label className={styles.label}>K<sub> p</sub> (default : 10):</label>
                                        <input type="text" value={this.state.k_p} onChange={this.handlekp} />
                                    </td>
                                    <td className={styles.td}>
                                        <label className={styles.label}>K<sub> i</sub> (default : 5) :</label>
                                        <input type="text" value={this.state.k_i} onChange={this.handleki} />
                                    </td>
                                    <td className={styles.td}>
                                        <label className={styles.label}>K<sub> d</sub> (default : 0.025):</label>
                                        <input type="text" value={this.state.k_d} onChange={this.handlekd} />
                                    </td>
                                    <td className={styles.button_td}>
                                        <button className={styles.button} type="submit" value="Submit">Submit</button>
                                    </td>
                                </tr>
                            </tbody>
                        </table>
                    </form>
                    <br></br>
                    <br></br>
                </div>
                <div>
                    <br>
                    </br>
                    <iframe width="450" height="260" className={styles.graph} src="https://thingspeak.com/channels/1856865/charts/6?bgcolor=%23ffffff&color=%23d62020&dynamic=true&results=75&title=Current+Angle&type=line&xaxis=Time%28+Difference+in+Time+scaled+10x+%29&yaxis=Angle+%28Degrees%29&yaxismin=0"></iframe>



                    <iframe width="450" height="260" className={styles.graph} src="https://thingspeak.com/channels/1856865/charts/5?bgcolor=%23ffffff&color=%23d62020&dynamic=true&results=75&title=Error&type=line&xaxis=Time%28+Difference+in+Time+scaled+10x+%29&yaxis=U"></iframe>



                    <iframe width="450" height="260" className={styles.graph} src="https://thingspeak.com/channels/1856865/charts/7?bgcolor=%23ffffff&color=%23d62020&dynamic=true&results=75&title=Power+Given&type=line&xaxis=Time%28+Difference+in+Time+scaled+10x+%29&yaxis=PWR"></iframe>
                </div>
            </div>

        );
    }
}

Graph.propTypes = {
    auth: PropTypes.object.isRequired,
};

const mapStateToProps = (state) => ({
    auth: state.auth,
});

export default connect(mapStateToProps)(Graph);
