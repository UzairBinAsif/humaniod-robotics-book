import React from 'react';
import Footer from '@theme-original/Footer';
import Chatbot from '../Chatbot';

export default function FooterWrapper(props) {
  return (
    <>
      <Chatbot />
      <Footer {...props} />
    </>
  );
}
