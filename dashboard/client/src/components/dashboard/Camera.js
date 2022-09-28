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

                    src="http://9a8d-2409-4070-2e92-cc2-7dbb-c484-9e8f-3fd5.ngrok.io/"
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
