import { Component } from "react";
import * as React from 'react';
import PropTypes from "prop-types";
import { connect } from "react-redux";




class Camera extends Component {

    render() {
        return (
            <div
                style={{
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center',
                    height: '100vh',
                }}
            >
                <iframe

                    src="http://c669-2409-4070-2bca-e542-3acd-3897-7e41-289f.ngrok.io/"
                    height="650"
                    width="900"
                    allowfullscreen="true"
                >
                </iframe>
            </div>

        );
    }
}

Camera.propTypes = {
    auth: PropTypes.object.isRequired,
};

const mapStateToProps = (state) => ({
    auth: state.auth,
});

export default connect(mapStateToProps)(Camera);
